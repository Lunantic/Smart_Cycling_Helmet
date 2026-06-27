/**
 ******************************************************************************
 * @file    hcsr04.h
 * @brief   HC-SR04 ultrasonic ranging sensor driver for STM32F103C8T6
 *          TRIG = PA1 (GPIO output), ECHO = PA0 (TIM2_CH1 input capture)
 *
 *          Hardware TIM2 input capture on CH1 (PA0) with digital filter
 *          to suppress breadboard parasitic coupling.  RT-Thread semaphore
 *          replaces busy-wait polling — thread sleeps until echo returns
 *          or 100ms timeout expires.
 *
 *          UART2: PA2(TX) / PA3(RX) @ 9600 baud (ASRPRO voice module)
 ******************************************************************************
 */

#ifndef __HCSR04_H
#define __HCSR04_H

#include "stm32f10x.h"
#include <rtthread.h>

/* ---- GPIO pins ---- */
#define HCSR04_TRIG_PORT    GPIOA
#define HCSR04_TRIG_PIN     GPIO_Pin_1
#define HCSR04_ECHO_PORT    GPIOA
#define HCSR04_ECHO_PIN     GPIO_Pin_0

/* ---- TIM2 config: APB1=36MHz, timer x2 = 72MHz, PSC=71 → 1MHz (1us) ---- */
#define HCSR04_TIM          TIM2
#define HCSR04_TIM_CLK      RCC_APB1Periph_TIM2
#define HCSR04_TIM_PSC      71          /* 72MHz / (71+1) = 1MHz  */
#define HCSR04_TIM_ARR      0xFFFF      /* max ~65.5ms = ~1130 cm  */

/* ---- Measurement params ---- */
#define HCSR04_MIN_DIST_CM          2.0f
#define HCSR04_MAX_DIST_CM          450.0f
#define HCSR04_ECHO_TIMEOUT_MS      100
#define HCSR04_MEASURE_PERIOD_MS    500

/* ---- Status enum ---- */
typedef enum {
    HCSR04_OK = 0,
    HCSR04_TIMEOUT,           /* semaphore wait expired (100ms)            */
    HCSR04_NO_ECHO,           /* TIM2 overflow — echo never returned       */
    HCSR04_OUT_OF_RANGE_LOW,  /* distance < 2 cm                          */
    HCSR04_OUT_OF_RANGE_HIGH, /* distance > 450 cm                        */
} hcsr04_status_t;

/* ---- Function prototypes ---- */
void HCSR04_Init(void);
void HCSR04_Trigger(void);
hcsr04_status_t HCSR04_GetDistance(float *distance_cm);
void HCSR04_IRQHandler(void);
void UART2_Init(void);
void UART2_SendString(const char *str);

#endif /* __HCSR04_H */
