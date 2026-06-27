/**
 ******************************************************************************
 * @file    voice.h
 * @brief   ASRPRO voice module driver — UART2 bidirectional protocol
 *
 *          STM32 → ASRPRO:  "D\r\n"       (obstacle < 2m → "小心行驶")
 *                           "V7\r\n"      (→ ASRPRO实际播报"右转", Voice_SendCmd内部修正)
 *                           "V8\r\n"      (→ ASRPRO实际播报"左转", Voice_SendCmd内部修正)
 *
 *          ASRPRO → STM32:  "C1\r\n"      (语音"打开左转灯" → 左灯亮)
 *                           "C2\r\n"      (语音"关闭左转灯" → 左灯灭)
 *                           "C3\r\n"      (语音"打开右转灯" → 右灯亮)
 *                           "C4\r\n"      (语音"关闭右转灯" → 右灯灭)
 *
 *          UART2: PA2=TX, PA3=RX, 9600-8-N-1, RXNE interrupt
 ******************************************************************************
 */

#ifndef __VOICE_H
#define __VOICE_H

#include "stm32f10x.h"

/* ---- 语音ID (与天问Block中配置一致) ---- */
#define VOICE_ID_PROXIMITY      5   /* "小心行驶" (收到"D"触发)   */
#define VOICE_ID_LEFT_ON        7   /* "左转" (调用V7, Voice_SendCmd内部修正为V8) */
#define VOICE_ID_RIGHT_ON       8   /* "右转" (调用V8, Voice_SendCmd内部修正为V7) */

/* ---- 接近报警阈值 ---- */
#define VOICE_PROXIMITY_CM      200 /* 距离 < 2m 触发语音报警    */

/* ---- ASRPRO 发来的命令枚举 ---- */
typedef enum {
    VOICE_CMD_NONE = 0,
    VOICE_CMD_LEFT_ON,      /* C1: 左转灯亮      */
    VOICE_CMD_LEFT_OFF,     /* C2: 左转灯灭      */
    VOICE_CMD_RIGHT_ON,     /* C3: 右转灯亮      */
    VOICE_CMD_RIGHT_OFF,    /* C4: 右转灯灭      */
} voice_cmd_t;

/* ---- 全局变量 (生产者写, voice线程消费) ---- */
extern volatile uint8_t   g_voice_dist_updated;  /* 距离数据更新标志      */
extern volatile uint16_t  g_voice_dist_cm;       /* 最新距离值 (cm)       */

/* ---- 函数声明 ---- */
void Voice_UART_RX_Init(void);                   /* USART2 RXNE中断使能   */
void Voice_UART_IRQHandler(void);                /* 中断: 填行缓冲         */
uint8_t Voice_CheckLine(void);                   /* 轮询: 返回1=有完整行   */
voice_cmd_t Voice_ParseLine(const char *line);   /* 解析 "C1"~"C4"        */
voice_cmd_t Voice_GetCommand(void);              /* 取内部缓冲的解析结果   */
void Voice_SendCmd(const char *cmd);             /* 发送协议字符串         */

#endif /* __VOICE_H */
