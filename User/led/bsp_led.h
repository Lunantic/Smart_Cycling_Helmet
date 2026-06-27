/**
 ******************************************************************************
 * @file    bsp_led.h
 * @brief   LED 引脚定义
 *
 *          灯带由 ws2812.c 驱动:
 *            PA8  — 左灯带数据线 (单线协议, 10 LEDs)
 *            PA11 — 右灯带数据线 (单线协议, 10 LEDs)
 *
 *          心跳 LED 为独立 GPIO:
 *            PB5  — 系统心跳指示 (推挽输出, 低电平亮)
 ******************************************************************************
 */

#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32f10x.h"

/* ---- 心跳 LED (PB5, 独立于 WS2812 灯带) ---- */
#define LED1_PORT       GPIOB
#define LED1_PIN        GPIO_Pin_5
#define LED1_CLK        RCC_APB2Periph_GPIOB

#define LED1_ON         GPIO_ResetBits(LED1_PORT, LED1_PIN)   /* 低电平亮 */
#define LED1_OFF        GPIO_SetBits(LED1_PORT, LED1_PIN)     /* 高电平灭 */

void LED_GPIO_Config(void);

#endif /* __BSP_LED_H */
