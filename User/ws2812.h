/**
 ******************************************************************************
 * @file    ws2812.h
 * @brief   双独立灯带驱动 — 单线可寻址智能LED (类WS2812协议)
 *
 *          左灯带: PA8 (DIN), 10个灯珠, 级联
 *          右灯带: PA11 (DIN), 10个灯珠, 级联
 *
 *          协议: 单极性归零码, 每灯24bit GRB数据
 ******************************************************************************
 */

#ifndef __WS2812_H
#define __WS2812_H

#include "stm32f10x.h"

/* ---- 灯带引脚 ---- */
#define WS2812_PORT          GPIOA
#define WS2812_PORT_CLK      RCC_APB2Periph_GPIOA
#define WS2812_LEFT_PIN      GPIO_Pin_8      /* PA8 = 左灯带数据线   */
#define WS2812_RIGHT_PIN     GPIO_Pin_11     /* PA11 = 右灯带数据线  */

/* ---- 灯带规格 ---- */
#define WS2812_NUM_LEDS      10              /* 每条灯带 10 个灯珠   */

/* ---- 24bit GRB 颜色 (G[23:16] R[15:8] B[7:0]) ---- */
#define WS2812_COLOR_OFF       0x000000UL   /* 全灭                      */

/* ---- 低亮度基础色 (~25% 占空比, 省电) ---- */
#define WS2812_COLOR_RED       0x003C00UL   /* 红  (R=60, G=0,   B=0)   */
#define WS2812_COLOR_GREEN     0x3C0000UL   /* 绿  (R=0,  G=60,  B=0)   */
#define WS2812_COLOR_BLUE      0x00003CUL   /* 蓝  (R=0,  G=0,   B=60)  */
#define WS2812_COLOR_ORANGE    0x283C00UL   /* 橙  (R=60, G=40,  B=0)   */
#define WS2812_COLOR_YELLOW    0x3C3C00UL   /* 黄  (R=60, G=60,  B=0)   */
#define WS2812_COLOR_CYAN      0x3C003CUL   /* 青  (R=0,  G=60,  B=60)  */
#define WS2812_COLOR_PURPLE    0x1E003CUL   /* 紫  (R=0,  G=30,  B=60)  */
#define WS2812_COLOR_PINK      0x3C1428UL   /* 粉  (R=20, G=60,  B=40)  */

/* ---- 炫彩流水灯 8 色调色板 (低亮度) ---- */
#define RAINBOW_COLORS  8
extern const uint32_t rainbow_palette[RAINBOW_COLORS];

/* ---- 灯带选择 ---- */
typedef enum {
    STRIP_LEFT  = 0,
    STRIP_RIGHT = 1
} strip_id_t;

/* ---- API ---- */
void ws2812_init(void);                           /* 初始化 PA8/PA11             */
void ws2812_clear(void);                          /* 两灯带全灭, 立即发送         */

void ws2812_set_strip(strip_id_t id, uint32_t color_grb);  /* 整条灯带设同色     */
void ws2812_set_led(strip_id_t id, uint8_t n, uint32_t color_grb); /* 单灯珠设色  */
void ws2812_send_strip(strip_id_t id);            /* 单灯带发送                  */
void ws2812_send_both(void);                      /* 双灯带一起发送              */

void ws2812_apply_rainbow(strip_id_t id, uint8_t step); /* 彩虹流水效果应用到灯带  */
void ws2812_prep_rainbow_off(strip_id_t active, strip_id_t other, uint8_t step); /* 准备彩虹+灭 */
void ws2812_apply_alarm(void);                    /* 报警: 双灯带全红灯           */

#endif /* __WS2812_H */
