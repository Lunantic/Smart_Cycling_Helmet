/**
 ******************************************************************************
 * @file    esp_bridge.h
 * @brief   ESP32 WiFi bridge — USART3 helmet telemetry forward
 *
 *          STM32 → ESP32:  "LAT:xx,LON:xx,TIME:xx,STATUS:x,SPEED:xx\r\n"
 *          (parsed by PC wifi_reader.py key-value protocol)
 *
 *          USART3: PB10=TX, PB11=RX, 115200-8-N-1, TX+RX with RXNE interrupt
 ******************************************************************************
 */

#ifndef __ESP_BRIDGE_H
#define __ESP_BRIDGE_H

#include "stm32f10x.h"
#include "gps.h"
#include "mpu6050.h"   /* for bike_status_t */

/* ---- PC command buffer (shared via ESP_CheckCommand) ---- */
#define ESP_CMD_BUF_SIZE   64
extern volatile uint8_t  g_esp_cmd_pending;   /* 1 = new command available    */
extern volatile char     g_esp_cmd[ESP_CMD_BUF_SIZE];  /* '\0' terminated    */

/* ---- Function prototypes ---- */
void     ESP_UART_Init(void);                                          /* USART3 PB10/PB11, 115200, TX+RX    */
void     ESP_SendLine(const char *str);                                /* Send string + "\r\n"                */
void     ESP_SendHelmetData(const gps_data_t *gps, bike_status_t st); /* Send full telemetry (GPS + status)  */
void     ESP_UART_IRQHandler(void);                                    /* USART3 RXNE ISR handler             */
uint8_t  ESP_CheckCommand(void);                                       /* Poll PC command (1 = new cmd ready) */

#endif /* __ESP_BRIDGE_H */
