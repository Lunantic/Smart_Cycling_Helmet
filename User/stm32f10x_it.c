/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <rtthread.h>
#include <rthw.h>
#include "hcsr04.h"
#include "gps.h"
#include "voice.h"
#include "esp_bridge.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    NVIC_SystemReset();
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}



/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    rt_os_tick_callback();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles TIM2 global interrupt request.
  *         HC-SR04 echo pulse measurement via input capture CH1 (PA0).
  * @param  None
  * @retval None
  */
/**
  * @brief  This function handles TIM2 global interrupt request.
  *         HC-SR04 echo pulse measurement via input capture CH1 (PA0).
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
    rt_interrupt_enter();
    HCSR04_IRQHandler();
    rt_interrupt_leave();
}

/**
  * @brief  This function handles USART1 global interrupt request.
  *         GPS NEO-6M NMEA sentence reception @ 9600 baud.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    rt_interrupt_enter();
    GPS_UART_IRQHandler();
    rt_interrupt_leave();
}

/**
  * @brief  This function handles USART2 global interrupt request.
  *         ASRPRO voice module — command reception @ 9600 baud.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
    rt_interrupt_enter();
    Voice_UART_IRQHandler();
    rt_interrupt_leave();
}

/**
  * @brief  This function handles USART3 global interrupt request.
  *         ESP32 WiFi bridge — PC command reception @ 115200 baud.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
    rt_interrupt_enter();
    ESP_UART_IRQHandler();
    rt_interrupt_leave();
}

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
