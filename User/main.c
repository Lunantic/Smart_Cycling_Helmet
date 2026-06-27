/**
 ******************************************************************************
 * @file    main.c
 * @brief   Smart Cycling System — STM32F103C8T6 + RT-Thread Nano
 *          1.8" TFT (ST7735R) + MPU6050 (I2C1 PB6/PB7)
 *          + HC-SR04 ultrasonic (TIM2_CH1 input capture)
 *          + GPS NEO-6M (USART1, 9600 baud)
 *          + Turn/Alarm LEDs (PA8 left, PA11 right)
 *
 *          All blocking busy-waits eliminated:
 *          - HC-SR04 uses TIM2 hardware input capture + semaphore
 *          - MPU6050 I2C uses reduced timeout (500us per event)
 *          - TFT updates throttled to 5 Hz (200ms)
 *
 * Screen flow:
 *   [Splash ~1.5s]  →  [Status screen]
 *   icon + title       title + "当前状态: 正常/摔倒!"
 *
 * Pin map:
 *   TFT:      SCL=PA5, SDA=PA7, CS=PA4, RST=PB0, DC=PB1, BL=PA12
 *   MPU6050:  SCL=PB6, SDA=PB7  (I2C1)
 *   L STRIP:  PA8  (左灯带数据线, WS2812单线协议, 10颗灯珠级联)
 *   R STRIP:  PA11 (右灯带数据线, WS2812单线协议, 10颗灯珠级联)
 *   SYS LED:  PB5  (心跳指示灯)
 *   HC-SR04:  TRIG=PA1, ECHO=PA0 (TIM2_CH1)
 *   GPS:      TX=PA9,  RX=PA10 (USART1)
 *   UART2:    TX=PA2,  RX=PA3  (9600-8-N-1, ASRPRO voice module)
 *   ESP32:    TX=PB10, RX=PB11 (USART3 115200)
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "bsp_led.h"
#include "Lcd_Driver.h"
#include "LCD_Config.h"
#include "GUI.h"
#include "boot_logo.h"
#include "text_logo.h"
#include "mpu6050.h"
#include "hcsr04.h"
#include "gps.h"
#include "voice.h"
#include "ws2812.h"
#include "esp_bridge.h"
#include <rtthread.h>
#include <stdio.h>
#include <string.h>

/* ================================================================
 *  Module enable switches — set to 0 to disable a module
 *
 *  For testing only GPS + ESP32, disable ASRPRO and HC-SR04:
 *    MODULE_ASRPRO_ENABLE  0
 *    MODULE_HCSR04_ENABLE  0
 *  MPU6050 has auto-detect (WHO_AM_I), safe to keep enabled.
 * ================================================================ */
#define MODULE_MPU6050_ENABLE   1   /* 0 = skip MPU6050 init entirely       */
#define MODULE_HCSR04_ENABLE    1   /* 0 = skip HC-SR04 ultrasonic thread   */
#define MODULE_ASRPRO_ENABLE    1   /* 0 = skip UART2 + voice thread        */

/* ---- Thread config ---- */
/*
 * Stack sizing notes (Cortex-M3, no FPU, newlib-nano):
 *   - sprintf with %f adds ~500 bytes of stack
 *   - TFT GUI operations add ~300 bytes
 *   - I2C/SPI driver calls add ~200 bytes
 *   - All sizes have 30-50% safety margin
 */
#define PRIO_SENSOR      20
#define PRIO_TURN_LED    22
#define PRIO_HEARTBEAT   26
#define PRIO_ULTRASONIC  24
#define PRIO_GPS         18
#define PRIO_VOICE       19

#define STK_SENSOR       2560   /* MPU6050 + TFT + sprintf(%f)             */
#define STK_TURN_LED     1024   /* ws2812 buffer + send                     */
#define STK_HEARTBEAT    512    /* LED1_ON/OFF only                        */
#define STK_ULTRASONIC   1536   /* sprintf(%f) + HCSR04_GetDistance        */
#define STK_GPS          2560   /* sprintf(%f) × 10 + 256-byte local buf   */
#define STK_VOICE        1536   /* sprintf(%f) + line buffer + RX parse    */
#define TSLICE           5

/* ---- Turn-signal LEDs (ws2812 protocol: PA8=left DIN, PA11=right DIN) ---- */

/* ---- Proximity alarm threshold (ultrasonic) ---- */
#define PROXIMITY_THRESHOLD_CM   200.0f   /* alarm when obstacle < 2m        */
#define PROXIMITY_CLEAR_CM       220.0f   /* hysteresis: clear alarm > 2.2m  */

/* ---- Globals shared with turn_led thread ---- */
volatile uint8_t g_proximity_alarm  = 0;    /* 1 = obstacle too close          */
volatile float   g_ultrasonic_dist  = 0.0f; /* latest valid distance (cm)      */

/* ---- Globals for voice thread (STM32 ↔ ASRPRO) ---- */
volatile uint8_t  g_voice_dist_updated = 0;       /* distance data updated flag  */
volatile uint16_t g_voice_dist_cm      = 0;       /* latest distance (cm)        */

/* ---- Globals for voice → turn_led communication ---- */
volatile uint8_t  g_voice_cmd         = 0;       /* voice_cmd_t from ASRPRO      */
volatile uint8_t  g_voice_cmd_pending = 0;       /* 1 = new command for turn_led */

/* ---- Globals for sensor → voice turn announcement ---- */
volatile uint8_t  g_turn_announce = 0;           /* 0=none, 1=左转, 2=右转        */

/* ---- Forward declarations ---- */
static void splash_show(void);
static void status_screen_init(void);
static void status_tft_update(bike_status_t st, float accel_mag,
                              const gps_data_t *gps);
static void sensor_thread_entry(void *param);
static void turn_led_thread_entry(void *param);
static void heartbeat_thread_entry(void *param);
static void ultrasonic_thread_entry(void *param);
static void gps_thread_entry(void *param);
static void voice_thread_entry(void *param);


/* ================================================================
 *  tft_blit — draw a RGB565 bitmap to TFT at (x, y)
 *
 *  Driver quirk: Lcd_SetRegion internally adds +2 to x_end and +1
 *  to y_end.  We subtract those so the actual hardware window equals
 *  the image dimensions exactly — otherwise pixels misalign.
 *  This is the SAME logic as the original, tested splash_show().
 * ================================================================ */
static void tft_blit(u16 x, u16 y, u16 w, u16 h, const unsigned short *px)
{
    u32 i;
    Lcd_SetRegion(x, y, x + w - 3, y + h - 2);
    Lcd_WriteIndex(0x2C);
    for (i = 0; i < (u32)w * (u32)h; i++)
        LCD_WriteData_16Bit(px[i]);
}

/* Fill a rectangle with a solid colour (compensates driver offsets) */
static void tft_fill(u16 x, u16 y, u16 w, u16 h, u16 color)
{
    u32 i;
    Lcd_SetRegion(x, y, x + w - 3, y + h - 2);
    Lcd_WriteIndex(0x2C);
    for (i = 0; i < (u32)w * (u32)h; i++)
        LCD_WriteData_16Bit(color);
}


/* ================================================================
 *  Splash screen  (main thread only — no races)
 * ================================================================ */
static void splash_show(void)
{
    u16 ic_x = (128 - BOOT_LOGO_WIDTH) / 2;   /* 28 */

    Lcd_Clear(WHITE);

    /* Icon 72×72, Y=31 */
    tft_blit(ic_x, 31, BOOT_LOGO_WIDTH, BOOT_LOGO_HEIGHT, gImage_boot_logo);

    /* Title "智能骑行系统" 128×24, Y=107 (31+72+4) */
    tft_blit(0, 107, TEXT_LOGO_WIDTH, TEXT_LOGO_HEIGHT, gImage_text_logo);
}


/* ================================================================
 *  Status screen — drawn once (background), then sensor thread
 *  updates only the "正常/摔倒!" value area.
 * ================================================================ */

static void status_screen_init(void)
{
    Lcd_Clear(WHITE);

    /* Title: "智能骑行系统" centered at top */
    tft_blit((128 - TEXT_LOGO_WIDTH) / 2, 4,
             TEXT_LOGO_WIDTH, TEXT_LOGO_HEIGHT, gImage_text_logo);

    /* Separator */
    Gui_DrawLine(8, 36, 119, 36, GRAY1);

    /* Status text ("当前状态:正常" / "当前状态:摔倒!") drawn by sensor thread */
}


/**
 * @brief  Update status + acceleration + GPS on TFT.
 *
 *         Detection uses two independent sensor axes:
 *         - Fall:  two-phase accel state machine (free-fall → impact)
 *         - Turn:  gyroscope Z axis with sustained-count + lockout
 *         GPS lat/lon displayed below acceleration.
 */
static void status_tft_update(bike_status_t st, float accel_mag,
                              const gps_data_t *gps)
{
    char buf[24];
    u8  *s;

    /* Line 1: Status — "当前状态:正常" or "当前状态:摔倒!" (GBK) */
    if (st == BIKE_STATUS_FALL)
        s = (u8 *)"\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC:\xCB\xA4\xB5\xB9!";
    else
        s = (u8 *)"\xB5\xB1\xC7\xB0\xD7\xB4\xCC\xAC:\xD5\xFD\xB3\xA3";

    tft_fill(0, 48, 128, 20, WHITE);
    Gui_DrawFont_GBK16(0, 50, 0x6B4D, WHITE, s);

    /* Line 2: "合加速度:" */
    tft_fill(0, 76, 128, 20, WHITE);
    Gui_DrawFont_GBK16(0, 78, 0x6B4D, WHITE,
                       (u8 *)"\xBA\xCF\xBC\xD3\xCB\xD9\xB6\xC8:");
    sprintf(buf, "%.2f g", (double)accel_mag);
    Gui_DrawFont_GBK16(72, 78, 0x6B4D, WHITE, (u8 *)buf);

    /* Line 3: "纬度:" + value  (GBK: CE B3 = 纬, B6 C8 = 度) */
    tft_fill(0, 98, 128, 20, WHITE);
    Gui_DrawFont_GBK16(0, 100, 0x6B4D, WHITE,
                       (u8 *)"\xCE\xB3\xB6\xC8:");
    if (gps != RT_NULL && gps->valid)
    {
        sprintf(buf, "%.4f", gps->latitude);
        Gui_DrawFont_GBK16(32, 100, 0x6B4D, WHITE, (u8 *)buf);
    }
    else
    {
        Gui_DrawFont_GBK16(32, 100, 0x6B4D, WHITE, (u8 *)"--");
    }

    /* Line 4: "经度:" + value  (GBK: BE AD = 经, B6 C8 = 度) */
    tft_fill(0, 118, 128, 20, WHITE);
    Gui_DrawFont_GBK16(0, 120, 0x6B4D, WHITE,
                       (u8 *)"\xBE\xAD\xB6\xC8:");
    if (gps != RT_NULL && gps->valid)
    {
        sprintf(buf, "%.4f", gps->longitude);
        Gui_DrawFont_GBK16(32, 120, 0x6B4D, WHITE, (u8 *)buf);
    }
    else
    {
        Gui_DrawFont_GBK16(32, 120, 0x6B4D, WHITE, (u8 *)"--");
    }
}


/* ================================================================
 *  Thread implementations
/* Sensor + display thread — samples @ 100Hz (10ms), continuously runs
 * two-phase fall detection + turn detection with lockout.
 * TFT is refreshed at ~5 Hz (every 200ms) to avoid SPI bus
 * congestion.  Detection still runs at 100 Hz.
 *
 * When MPU6050 is absent, sensor detection is skipped but GPS
 * lat/lon is still displayed.  This allows running the system
 * with only the GPS module connected (e.g. outdoor testing). */
#if 1  /* ---- Sensor/TFT thread ---- */
static void sensor_thread_entry(void *param)
{
    int8_t      last_st       = -1;
    float       last_accel    = 999.0f;
    uint8_t     last_has_fix  = 0;
    uint8_t     tick          = 0;
    uint8_t     mpu_ok        = 1;
    uint8_t     gps_changed   = 0;
    gps_data_t  gps_cache;
    uint8_t     gps_has_fix   = 0;

    rt_thread_mdelay(200);

#if MODULE_MPU6050_ENABLE
    /* ---- Try init MPU6050; continue without it on failure ---- */
    if (MPU6050_Init() != 0)
    {
        mpu_ok = 0;
        /* Show one-time warning, then keep running for GPS display */
        tft_fill(0, 48, 128, 40, WHITE);
        Gui_DrawFont_GBK16(0, 50, RED, WHITE,
                           (u8 *)"\xB4\xAB\xB8\xD0\xC6\xF7\xB4\xED\xCE\xF3");
        Gui_DrawFont_GBK16(0, 78, RED, WHITE,
                           (u8 *)"\xC7\xEB\xBC\xEC\xB2\xE9I2C\xBD\xD3\xCF\xDF");
        rt_thread_mdelay(3000);  /* Let user read warning */
        status_screen_init();    /* Re-draw clean layout */
    }
#else
    mpu_ok = 0;  /* MPU6050 disabled at compile time */
#endif

    while (1)
    {
#if MODULE_MPU6050_ENABLE
        /* ---- MPU6050 detection (skip if absent) ---- */
        if (mpu_ok)
        {
            MPU6050_Update();           /* detection @ 100 Hz    */
        }
#endif
        tick++;

        /* ---- Sensor heartbeat to PC (every 500ms, no GPS needed) ---- */
        if ((tick % 50) == 0)
        {
            char hb[32];
            sprintf(hb, "S:ACC:%.2f,GYZ:%.1f,STA:%d",
                    (double)g_accel_mag, (double)g_gyro_z, (int)g_bike_status);
            ESP_SendLine(hb);
        }

        /* ---- Cache latest GPS fix ---- */
        if (g_gps_updated)
        {
            gps_cache   = g_gps_data;
            gps_has_fix = 1;
            gps_changed = 1;
            g_gps_updated = 0;

            /* Forward to ESP32 for WiFi upload (GPS + fall status) */
            ESP_SendHelmetData(&gps_cache, g_bike_status);
        }

        /* ---- ESP32 PC commands (moved here from voice thread for
         *      independence — works even without ASRPRO module) ---- */
        if (ESP_CheckCommand())
        {
            const char *cmd = (const char *)g_esp_cmd;
            g_esp_cmd_pending = 0;

            if (strcmp(cmd, "RESET") == 0)
            {
                rt_thread_mdelay(50);
                NVIC_SystemReset();
            }
            else if (strcmp(cmd, "FALL_EVENT") == 0)
            {
                g_bike_status = BIKE_STATUS_FALL;
            }
            else if (strcmp(cmd, "LEFT_ON") == 0)
            {
                g_voice_cmd         = VOICE_CMD_LEFT_ON;
                g_voice_cmd_pending = 1;
            }
            else if (strcmp(cmd, "LEFT_OFF") == 0)
            {
                g_voice_cmd         = VOICE_CMD_LEFT_OFF;
                g_voice_cmd_pending = 1;
            }
            else if (strcmp(cmd, "RIGHT_ON") == 0)
            {
                g_voice_cmd         = VOICE_CMD_RIGHT_ON;
                g_voice_cmd_pending = 1;
            }
            else if (strcmp(cmd, "RIGHT_OFF") == 0)
            {
                g_voice_cmd         = VOICE_CMD_RIGHT_OFF;
                g_voice_cmd_pending = 1;
            }
        }

        /* Refresh TFT on any of:
         *   1. Sensor status changed (fall / turn / normal)
         *   2. GPS data changed (new fix arrived)
         *   3. Every 200ms (20 ticks) — guarantees periodic GPS update */
        if (g_bike_status != last_st ||
            gps_changed ||
            gps_has_fix != last_has_fix ||
            (tick >= 20))
        {
            status_tft_update(g_bike_status, g_accel_mag,
                              gps_has_fix ? &gps_cache : RT_NULL);

            /* Trigger voice announcement on turn (skip init: last_st=-1) */
            if (last_st >= 0)
            {
                if (g_bike_status == BIKE_STATUS_LEFT_TURN)
                    g_turn_announce = 1;
                else if (g_bike_status == BIKE_STATUS_RIGHT_TURN)
                    g_turn_announce = 2;
            }

            last_st      = (int8_t)g_bike_status;
            last_accel   = g_accel_mag;
            last_has_fix = gps_has_fix;
            gps_changed  = 0;
            tick = 0;
        }

        rt_thread_mdelay(10);   /* 10ms = base tick for debounce counters */
    }
}
#endif /* Sensor/TFT thread */


/* Turn LED thread — WS2812 addressable LED strips
 *
 *  左灯带: PA8 (DIN), 10颗灯珠级联
 *  右灯带: PA11 (DIN), 10颗灯珠级联
 *
 * State machine:
 *   IDLE         → wait for triggers
 *   TURN_STEADY  → 2s continuous flowing rainbow, then IDLE
 *   PROX_FLASH   → 100ms × 3 blinks, repeat while obstacle present
 *   FALL         → 100ms × 3 blinks, highest priority
 *
 * Fall always interrupts any other sequence.
 */
/* ---- LED strip animation helpers ---------------------------------------- */
/*
 * 炫彩流水灯 — 每条灯带 10 灯珠, 8 色彩虹调色板循环流动.
 * 左转: 左灯带连续流水亮2秒, 右灯带灭 (不闪烁)
 * 右转: 右灯带连续流水亮2秒, 左灯带灭 (不闪烁)
 * 报警: 双灯带低亮度红灯 100ms × 3 次闪烁
 * 摔倒: 同报警模式 (100ms × 3 次)
 *
 * 所有颜色均 ~25% 占空比, 大幅降低功耗.
 */

/*
 * direction: 0=both(alarm), 1=left turn, 2=right turn
 * onoff:     0=OFF, 1=ON
 */
static void led_strip_apply_state(uint8_t direction, uint8_t onoff)
{
    if (!onoff)
    {
        /* OFF — all dark */
        ws2812_clear();
        return;
    }

    /* ON */
    if (direction == 0)
    {
        /* Alarm/Fall — both strips low-brightness RED */
        ws2812_apply_alarm();
    }
    else if (direction == 1)
    {
        /* Left turn — left rainbow flow, right off */
        ws2812_set_strip(STRIP_RIGHT, WS2812_COLOR_OFF);
        ws2812_send_strip(STRIP_RIGHT);
    }
    else  /* direction == 2 */
    {
        /* Right turn — right rainbow flow, left off */
        ws2812_set_strip(STRIP_LEFT, WS2812_COLOR_OFF);
        ws2812_send_strip(STRIP_LEFT);
    }
}


#if 1  /* ---- Turn LED thread ---- */
static void turn_led_thread_entry(void *param)
{
    typedef enum {
        S_IDLE = 0,
        S_TURN_STEADY,
        S_PROX_FLASH,
        S_FALL
    } led_state_t;

    led_state_t state       = S_IDLE;
    uint8_t     direction   = 0;   /* 1=left, 2=right */
    uint8_t     count       = 0;
    uint8_t     prox_active = 0;
    uint8_t     voice_led   = 0;   /* 0=none, 1=left, 2=right (voice override) */
    uint8_t     flow_step   = 0;   /* rainbow animation frame counter */

    #define TURN_STEADY_MS   2000  /* continuous flowing rainbow for 2s          */
    #define ALARM_ON_MS      100   /* flash ON  time: 100ms                     */
    #define ALARM_OFF_MS     100   /* flash OFF time: 100ms                     */
    #define ALARM_BLINK_NUM  5     /* 5 blinks per alarm cycle                  */
    #define FLOW_FRAME_MS    120   /* rainbow animation frame interval (slower) */

    /* GPIO + ws2812 already initialised by LED_GPIO_Config() in main() */

    /* First action: force both strips OFF regardless of prior state */
    ws2812_clear();
    rt_thread_mdelay(10);

    while (1)
    {
        /* ============================================================
         *  FALL — highest priority, 100ms × 3 blinks
         * ============================================================ */
        if (g_bike_status == BIKE_STATUS_FALL)
        {
            state     = S_FALL;
            direction = 0;
            count     = 0;
            for (count = 0; count < ALARM_BLINK_NUM; count++)
            {
                led_strip_apply_state(0, 1);   /* RED ON  */
                rt_thread_mdelay(ALARM_ON_MS);
                led_strip_apply_state(0, 0);   /* OFF     */
                rt_thread_mdelay(ALARM_OFF_MS);
            }
            continue;
        }

        /* ============================================================
         *  VOICE — immediate ON/OFF with rainbow, preempts others
         * ============================================================ */
        if (g_voice_cmd_pending)
        {
            uint8_t vcmd = g_voice_cmd;
            g_voice_cmd_pending = 0;

            switch (vcmd)
            {
            case VOICE_CMD_LEFT_ON:
                ws2812_prep_rainbow_off(STRIP_LEFT, STRIP_RIGHT, flow_step++);
                ws2812_send_both();
                voice_led = 1;
                break;
            case VOICE_CMD_LEFT_OFF:
                ws2812_clear();
                voice_led = 0;
                break;
            case VOICE_CMD_RIGHT_ON:
                ws2812_prep_rainbow_off(STRIP_RIGHT, STRIP_LEFT, flow_step++);
                ws2812_send_both();
                voice_led = 2;
                break;
            case VOICE_CMD_RIGHT_OFF:
                ws2812_clear();
                voice_led = 0;
                break;
            default:
                break;
            }
            state     = S_IDLE;
            direction = 0;
            count     = 0;
            continue;
        }

        /* ============================================================
         *  IDLE — wait for sensor / voice triggers
         * ============================================================ */
        if (state == S_IDLE)
        {
            /* Check proximity alarm (latch on rising edge) */
            if (g_proximity_alarm && !prox_active)
            {
                prox_active = 1;
                state       = S_PROX_FLASH;
                count       = 0;
            }
            if (!g_proximity_alarm)
                prox_active = 0;

            /* Check turn signals (sensor detected) */
            if (g_bike_status == BIKE_STATUS_LEFT_TURN)
            {
                voice_led = 0;           /* sensor overrides voice */
                state     = S_TURN_STEADY;
                direction = 1;
                count     = 0;
            }
            else if (g_bike_status == BIKE_STATUS_RIGHT_TURN)
            {
                voice_led = 0;
                state     = S_TURN_STEADY;
                direction = 2;
                count     = 0;
            }
            else
            {
                /* Maintain voice LED state with rainbow animation */
                if (voice_led == 0)
                {
                    led_strip_apply_state(0, 0);  /* all off */
                }
                else if (voice_led == 1)
                {
                    ws2812_prep_rainbow_off(STRIP_LEFT, STRIP_RIGHT, flow_step++);
                    ws2812_send_both();
                }
                else  /* voice_led == 2 */
                {
                    ws2812_prep_rainbow_off(STRIP_RIGHT, STRIP_LEFT, flow_step++);
                    ws2812_send_both();
                }
                rt_thread_mdelay(FLOW_FRAME_MS);
                continue;
            }
        }

        /* ============================================================
         *  TURN_STEADY — continuous flowing rainbow for 2s, then all off
         *  不闪烁, 直接流水灯连续亮2秒
         * ============================================================ */
        if (state == S_TURN_STEADY)
        {
            {
                uint32_t elapsed = 0;
                strip_id_t active = (direction == 1) ? STRIP_LEFT : STRIP_RIGHT;
                strip_id_t other  = (direction == 1) ? STRIP_RIGHT : STRIP_LEFT;

                while (elapsed < TURN_STEADY_MS)
                {
                    if (g_voice_cmd_pending) break;
                    ws2812_prep_rainbow_off(active, other, flow_step++);
                    ws2812_send_both();
                    rt_thread_mdelay(FLOW_FRAME_MS);
                    elapsed += FLOW_FRAME_MS;
                }
            }
            /* All off after steady period (or voice interrupt) */
            led_strip_apply_state(0, 0);
            state     = S_IDLE;
            direction = 0;
            continue;
        }

        /* ============================================================
         *  PROX_FLASH — 100ms × 3 blinks, repeat while obstacle present
         * ============================================================ */
        if (state == S_PROX_FLASH)
        {
            if (!g_proximity_alarm)
            {
                led_strip_apply_state(0, 0);
                state       = S_IDLE;
                prox_active = 0;
                continue;
            }

            for (count = 0; count < ALARM_BLINK_NUM; count++)
            {
                if (!g_proximity_alarm) break;
                led_strip_apply_state(0, 1);   /* RED ON  */
                rt_thread_mdelay(ALARM_ON_MS);
                led_strip_apply_state(0, 0);   /* OFF     */
                rt_thread_mdelay(ALARM_OFF_MS);
            }
            continue;
        }
    }
}
#endif /* Turn LED thread */


/* Heartbeat LED (PB5) — 100ms on every 2s */
#if 1  /* ---- Heartbeat thread ---- */
static void heartbeat_thread_entry(void *param)
{
    while (1)
    {
        LED1_ON;  rt_thread_mdelay(100);
        LED1_OFF; rt_thread_mdelay(1900);
    }
}
#endif /* Heartbeat thread */


/* Ultrasonic thread — HC-SR04 sampling @ 2Hz → voice + alarm
 *
 *  Also drives g_proximity_alarm / g_ultrasonic_dist for the turn_led
 *  thread.  Hysteresis: alarm ON < 200cm, alarm OFF > 220cm.
 *
 *  Auto-disables after 5 consecutive failures (module disconnected).
 *  Thread then sleeps forever, releasing all CPU time.
 */
#if MODULE_HCSR04_ENABLE  /* ---- Ultrasonic thread ---- */
static void ultrasonic_thread_entry(void *param)
{
    float           dist;
    hcsr04_status_t status;
    uint8_t         fail_count = 0;

    rt_thread_mdelay(2000);   /* wait 2s: let 5V stabilise + module boot */

    while (1)
    {
        status = HCSR04_GetDistance(&dist);

        if (status == HCSR04_OK)
        {
            fail_count = 0;    /* reset fail counter on success */

            g_ultrasonic_dist = dist;

            /* Feed distance to voice thread (UART2 → ASRPRO) */
            g_voice_dist_cm      = (uint16_t)dist;
            g_voice_dist_updated = 1;

            if (dist < PROXIMITY_THRESHOLD_CM)
                g_proximity_alarm = 1;
            else if (dist > PROXIMITY_CLEAR_CM)
                g_proximity_alarm = 0;
        }
        else
        {
            g_proximity_alarm = 0;

            /* 5 consecutive failures → module absent, sleep forever */
            fail_count++;
            if (fail_count >= 5)
            {
                /* Thread sleeps forever — zero CPU, zero bus activity */
                while (1) { rt_thread_mdelay(60000); }
            }
        }

        rt_thread_mdelay(HCSR04_MEASURE_PERIOD_MS);
    }
}
#endif /* Ultrasonic thread */


/* ================================================================
 *  GPS thread — polls GPS NMEA lines, feeds sensor thread
 * ================================================================ */
static void gps_thread_entry(void *param)
{
    /* Wait for GPS module to stabilise (~2s cold start) */
    rt_thread_mdelay(2000);

    while (1)
    {
        /* ---- Feed NMEA lines to parser ---- */
        while (GPS_CheckLine())
        {
            /* Lines are being processed; GPS_CheckLine returns 1
             * for each sentence consumed.  Keep draining until the
             * line buffer is empty.
             *
             * When both $GPRMC and $GPGGA are received, GPS_ProcessLine
             * sets g_gps_updated=1.  The sensor thread reads g_gps_data
             * and clears the flag — this thread must NOT touch it. */
        }

        /* Yield CPU (10ms tick — GPS updates @ 1-10 Hz, no need to spin) */
        rt_thread_mdelay(10);
    }
}


/* ================================================================
 *  Voice thread — bidirectional ASRPRO communication
 *
 *  TX (STM32 → ASRPRO):
 *    Proximity alert: "D\r\n"    when dist < 200cm → "小心行驶"
 *    Command reply:   "V7\r\n" / "V8\r\n" / "V9\r\n" → confirmation.
 *
 *  RX (ASRPRO → STM32):
 *    Voice commands: "C1\r\n" (左转灯亮), "C2\r\n" (左转灯灭),
 *                    "C3\r\n" (右转灯亮), "C4\r\n" (右转灯灭).
 *
 *  Priority 19 — between GPS(18) and sensor(20).
 *  Sole owner of UART2 TX — no other thread writes to UART2.
 * ================================================================ */
#if MODULE_ASRPRO_ENABLE  /* ---- Voice thread ---- */
static void voice_thread_entry(void *param)
{
    voice_cmd_t cmd;
    uint8_t     prox_cooldown = 0;
    uint8_t     cmd_in_cycle = 0;   /* ≤1 cmd per 50ms poll cycle */
    #define PROX_COOLDOWN_TICKS  60  /* 60 × 50ms = 3s */

    /* Enable USART2 RX interrupt (UART2 already init'd by UART2_Init) */
    Voice_UART_RX_Init();

    rt_thread_mdelay(100);  /* Let ASRPRO boot (SD mount ~50ms) */

    while (1)
    {
        /* ==========================================================
         *  1. Handle received voice commands (ASRPRO → STM32)
         *
         *  ASRPRO sends N identical lines per command (snid stuck).
         *  We process ≤1 command per poll cycle — bursts collapse,
         *  while legitimate repeat commands (seconds apart) work.
         * ========================================================== */
        cmd_in_cycle = 0;   /* reset each 50ms poll */

        while (Voice_CheckLine())
        {
            if (cmd_in_cycle)
                continue;       /* only 1 command per poll cycle */

            cmd = Voice_GetCommand();

            if (cmd != VOICE_CMD_NONE)
                cmd_in_cycle = 1;

            switch (cmd)
            {
            case VOICE_CMD_LEFT_ON:     /* C1: 左转灯亮 */
                g_voice_cmd = VOICE_CMD_LEFT_ON;
                g_voice_cmd_pending = 1;
                break;

            case VOICE_CMD_LEFT_OFF:    /* C2: 左转灯灭 */
                g_voice_cmd = VOICE_CMD_LEFT_OFF;
                g_voice_cmd_pending = 1;
                break;

            case VOICE_CMD_RIGHT_ON:    /* C3: 右转灯亮 */
                g_voice_cmd = VOICE_CMD_RIGHT_ON;
                g_voice_cmd_pending = 1;
                break;

            case VOICE_CMD_RIGHT_OFF:   /* C4: 右转灯灭 */
                g_voice_cmd = VOICE_CMD_RIGHT_OFF;
                g_voice_cmd_pending = 1;
                break;

            default:
                break;
            }
        }

        /* ==========================================================
         *  2. Sensor turn announcement → V7/V8 (STM32 → ASRPRO)
         * ========================================================== */
        if (g_turn_announce)
        {
            if (g_turn_announce == 1)
                Voice_SendCmd("V7");    /* "左转" */
            else if (g_turn_announce == 2)
                Voice_SendCmd("V8");    /* "右转" */
            g_turn_announce = 0;
        }

        /* ==========================================================
         *  4. Send distance data (STM32 → ASRPRO)
         *     Only when obstacle is within 2 metres.
         *     3-second cooldown prevents repeated announcements.
         * ========================================================== */
        if (prox_cooldown > 0)
            prox_cooldown--;

        if (g_voice_dist_updated)
        {
            g_voice_dist_updated = 0;

            if (g_voice_dist_cm < VOICE_PROXIMITY_CM && prox_cooldown == 0)
            {
                Voice_SendCmd("D");     /* "小心行驶" */
                prox_cooldown = PROX_COOLDOWN_TICKS;
            }
        }

        rt_thread_mdelay(50);  /* 20 Hz polling */
    }
}
#endif /* Voice thread */


/* ================================================================
 *  Main — phased initialization
 *
 *  Phase 0 — LED MUST be first! (WS2812 PA8/PA11 before any GPIOA user)
 *  Phase 1 — Critical comms (UART2, GPS, ESP32)
 *  Phase 2 — Peripheral init (TFT, MPU6050, HC-SR04)
 *  Phase 3 — Thread creation (start scheduler tasks)
 *  Phase 4 — Main thread sleeps, yielding CPU to worker threads
 *
 *  All blocking/busy-wait operations have been eliminated:
 *    - HC-SR04 uses hardware input capture + RT-Thread semaphore
 *    - MPU6050 I2C uses hardware timeout + auto-reset
 * ================================================================ */
int main(void)
{
    rt_thread_t tid;
    volatile uint32_t dly;

    /* ============================================================
     *  Phase 0 — LED MUST be first!
     *            ws2812_init configures PA8/PA11 as output LOW
     *            BEFORE any other GPIOA user (UART2/GPS) enables
     *            the GPIOA clock.  Otherwise PA11 floats and the
     *            right LED strip latches random colour on power-up.
     * ============================================================ */
    LED_GPIO_Config();       /* PA8/PA11 ws2812 + PB5 heartbeat — FIRST! */

    /* ============================================================
     *  Phase 1 — Critical communication peripherals
     * ============================================================ */

#if MODULE_ASRPRO_ENABLE
    UART2_Init();            /* PA2/PA3, 9600, ASRPRO voice module  */

    /* Let UART2 TX line stabilise (~1 ms @ 72 MHz) */
    for (dly = 0; dly < 72000; dly++) __NOP();
#endif

    GPS_UART_Init();         /* PA9/PA10, USART1 9600, GPS NEO-6M  */
    rt_thread_mdelay(500);   /* wait for GPS module cold boot       */
    GPS_Set5Hz();            /* configure NEO-6M to 5Hz update rate */
    ESP_UART_Init();         /* PB10/PB11, USART3 115200, ESP32    */

    /* ============================================================
     *  Phase 2 — Peripheral hardware init
     * ============================================================ */

    LCD_GPIO_Init();
    Lcd_Init();
    LCD_LED_SET;             /* Backlight ON                       */
    splash_show();           /* Boot logo ~instant on SPI           */
    rt_thread_mdelay(1500);  /* Let user see splash                */
    status_screen_init();    /* Switch to status layout            */

#if MODULE_MPU6050_ENABLE
    MPU6050_I2C_Init();      /* PB6=SCL, PB7=SDA, 100kHz           */
#endif

#if MODULE_HCSR04_ENABLE
    HCSR04_Init();           /* PA1=TRIG, PA0=ECHO, 1us capture    */
#endif

    /* ============================================================
     *  Phase 3 — Create RT-Thread worker threads
     *            Higher priority = lower number
     * ============================================================ */

    /* GPS thread — NMEA parsing + serial output  [PRIO 18] */
    tid = rt_thread_create("gps",
                           gps_thread_entry, RT_NULL,
                           STK_GPS, PRIO_GPS, TSLICE);
    if (tid != RT_NULL) { rt_thread_startup(tid); }

    /* Sensor + TFT display thread  [PRIO 20] */
    tid = rt_thread_create("sensor",
                           sensor_thread_entry, RT_NULL,
                           STK_SENSOR, PRIO_SENSOR, TSLICE);
    if (tid != RT_NULL) { rt_thread_startup(tid); }

#if MODULE_ASRPRO_ENABLE
    /* Voice thread — ASRPRO bidirectional UART2  [PRIO 19] */
    tid = rt_thread_create("voice",
                           voice_thread_entry, RT_NULL,
                           STK_VOICE, PRIO_VOICE, TSLICE);
    if (tid != RT_NULL) { rt_thread_startup(tid); }
#endif

    /* Turn / alarm LED thread  [PRIO 22] */
    tid = rt_thread_create("turn_led",
                           turn_led_thread_entry, RT_NULL,
                           STK_TURN_LED, PRIO_TURN_LED, TSLICE);
    if (tid != RT_NULL) { rt_thread_startup(tid); }

#if MODULE_HCSR04_ENABLE
    /* Ultrasonic ranging thread  [PRIO 24] */
    tid = rt_thread_create("ultrasonic",
                           ultrasonic_thread_entry, RT_NULL,
                           STK_ULTRASONIC, PRIO_ULTRASONIC, TSLICE);
    if (tid != RT_NULL) { rt_thread_startup(tid); }
#endif

    /* System heartbeat LED thread  [PRIO 26] */
    tid = rt_thread_create("heartbeat",
                           heartbeat_thread_entry, RT_NULL,
                           STK_HEARTBEAT, PRIO_HEARTBEAT, TSLICE);
    if (tid != RT_NULL) { rt_thread_startup(tid); }

    /* ============================================================
     *  Phase 4 — Main thread yields CPU forever
     *            All real work is in the RT-Thread worker threads.
     *            Short sleep to keep scheduler responsive.
     * ============================================================ */
    while (1)
    {
        rt_thread_mdelay(500);
    }
}
