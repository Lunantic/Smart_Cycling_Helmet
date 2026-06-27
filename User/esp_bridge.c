/**
 ******************************************************************************
 * @file    esp_bridge.c
 * @brief   ESP32 WiFi bridge — USART3 helmet telemetry forward
 *
 *          STM32 → ESP32 over USART3 (PB10=TX, PB11=RX), 115200-8-N-1.
 *          Sends full telemetry (GPS + fall status + speed) whenever a new
 *          valid GPS fix arrives.
 *
 *          Format:  LAT:xx.xxxx,LON:xx.xxxx,TIME:HH:MM:SS,STATUS:x,SPEED:xx.x\r\n
 *          Example: LAT:31.2304,LON:121.4737,TIME:12:30:45,STATUS:0,SPEED:23.2\r\n
 *
 *          TX+RX with RXNE interrupt for bidirectional PC comms.
 *          Blocking send ~2ms per line at 115200 baud (~60 bytes),
 *          called from sensor thread (PRIO 20).
 ******************************************************************************
 */

#include "esp_bridge.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>


/* ================================================================
 *  Line buffer for interrupt-driven command reception
 *
 *  Same strategy as GPS/Voice: ISR fills line buffer, sets ready
 *  flag on '\n'.  Main thread polls ESP_CheckCommand().
 *  At 115200 baud: ~11 chars/ms, PC commands are short — 64-byte
 *  buffer is more than enough.
 * ================================================================ */
#define ESP_CMD_BUF_SIZE   64

static volatile uint8_t  esp_cmd_buf[ESP_CMD_BUF_SIZE];
static volatile uint8_t  esp_cmd_idx   = 0;
static volatile uint8_t  esp_cmd_ready = 0;

/* Global — accessed by other threads via ESP_GetCommand() */
volatile uint8_t  g_esp_cmd_pending = 0;
volatile char     g_esp_cmd[ESP_CMD_BUF_SIZE];


/* ================================================================
 *  ESP_UART_Init — USART3: PB10=TX, PB11=RX, 115200-8-N-1
 *
 *  TX + RX with RXNE interrupt for bidirectional PC comms.
 *  PB10 used to be TFT backlight; now freed (BL → PA12).
 * ================================================================ */
void ESP_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* ---- PB10 = USART3_TX (alternate function push-pull) ---- */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ---- PB11 = USART3_RX (pull-up: prevent noise from unpowered ESP32) ---- */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ---- USART3: 115200-8-N-1, TX + RX ---- */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    /* ---- Enable RXNE interrupt ---- */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* ---- NVIC: USART3 global interrupt, prio 3 (lower than GPS=1, Voice=2) ---- */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}


/* ================================================================
 *  ESP_SendLine — blocking string send with "\r\n"
 *
 *  Called ONLY from GPS thread (single writer — no race condition).
 *  At 115200 baud each byte takes ~87us, a full line ~2ms max.
 * ================================================================ */
void ESP_SendLine(const char *str)
{
    while (*str)
    {
        USART_SendData(USART3, (uint16_t)(*str++));
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
    USART_SendData(USART3, (uint16_t)'\r');
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, (uint16_t)'\n');
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}


/* ================================================================
 *  ESP_SendHelmetData — format and send full helmet telemetry
 *
 *  Output format (key-value, parsed by PC wifi_reader.py):
 *    LAT:xx.xxxx,LON:xx.xxxx,TIME:HH:MM:SS,STATUS:x,SPEED:xx.x
 *
 *  Fields:
 *    LAT    — decimal degrees, 4 decimal places  (double)
 *    LON    — decimal degrees, 4 decimal places  (double)
 *    TIME   — UTC time HH:MM:SS from GPS fix     (from gps->hour/min/sec)
 *    STATUS — 0 = normal riding, 1 = fall detected (from MPU6050)
 *    SPEED  — km/h, 1 decimal place              (converted from knots)
 *
 *  Sends ONLY when valid == 1 (GPS has fix).
 *  Called from sensor thread — safe to read global g_bike_status.
 * ================================================================ */
void ESP_SendHelmetData(const gps_data_t *gps, bike_status_t status)
{
    char buf[80];
    int  status_raw;
    float speed_kmh;

    if (gps == ((void *)0) || !gps->valid)
        return;   /* don't send invalid / no-fix data */

    /* STATUS: 0=normal, 1=fall (PC protocol expects this encoding) */
    status_raw = (status == BIKE_STATUS_FALL) ? 1 : 0;

    /* Convert knots → km/h (1 knot = 1.852 km/h) */
    speed_kmh = gps->speed * 1.852f;

    sprintf(buf, "LAT:%.4f,LON:%.4f,TIME:%02d:%02d:%02d,STATUS:%d,SPEED:%.1f",
            gps->latitude,
            gps->longitude,
            gps->hour, gps->minute, gps->second,
            status_raw,
            (double)speed_kmh);

    ESP_SendLine(buf);
}


/* ================================================================
 *  ESP_UART_IRQHandler — USART3 RXNE interrupt handler
 *
 *  Called from USART3_IRQHandler in stm32f10x_it.c.
 *  Fills line buffer; on '\n', NUL-terminates and sets
 *  esp_cmd_ready = 1.
 *
 *  PC commands (透传自 ESP32 TCP):
 *    RESET       — 系统复位
 *    FALL_EVENT  — 模拟摔倒事件
 *    LED_ON      — 手动开灯
 *    LED_OFF     — 手动关灯
 *    ...         — 扩展命令
 *
 *  WARNING: runs in ISR context — no blocking calls.
 * ================================================================ */
void ESP_UART_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART3);

        /* Line complete (LF received) */
        if (ch == '\n')
        {
            if (esp_cmd_idx > 0 &&
                esp_cmd_buf[esp_cmd_idx - 1] == '\r')
                esp_cmd_idx--;                       /* strip trailing CR     */
            esp_cmd_buf[esp_cmd_idx] = '\0';         /* NUL-terminate         */
            esp_cmd_idx  = 0;
            esp_cmd_ready = 1;                       /* signal main thread    */
        }
        /* Discard CR, skip buffer overflow */
        else if (ch == '\r')
        {
            /* nothing — LF will handle termination */
        }
        else if (esp_cmd_idx < ESP_CMD_BUF_SIZE - 1)
        {
            esp_cmd_buf[esp_cmd_idx++] = ch;
        }
        else
        {
            /* Buffer overflow — reset, wait for next line */
            esp_cmd_idx = 0;
        }
    }
}


/* ================================================================
 *  ESP_CheckCommand — poll for a complete PC command line
 *
 *  Returns 1 if a command was ready and copied to g_esp_cmd,
 *  0 otherwise.  The command is '\0' terminated.
 *
 *  Thread-safe: disables RXNE IRQ during copy.
 *  Call from any thread that needs to handle PC commands.
 * ================================================================ */
uint8_t ESP_CheckCommand(void)
{
    if (!esp_cmd_ready)
        return 0;

    /* Critical section: disable RXNE, copy, re-enable */
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    strcpy((char *)g_esp_cmd, (const char *)esp_cmd_buf);
    esp_cmd_ready = 0;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    g_esp_cmd_pending = 1;
    return 1;
}
