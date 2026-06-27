/**
 ******************************************************************************
 * @file    voice.c
 * @brief   ASRPRO voice module driver implementation
 *
 *          Communication pattern (same as GPS NMEA):
 *            USART2 RXNE ISR → line buffer → voice_line_ready flag
 *            voice thread polls Voice_CheckLine() → Voice_ParseLine()
 *
 *          Protocol:
 *            STM32 → ASRPRO:  "D\r\n"    (obstacle < 2m)
 *                             "V7\r\n"  (→ ASRPRO实际播报"右转")
 *                             "V8\r\n"  (→ ASRPRO实际播报"左转")
 *                             注意: Voice_SendCmd() 内部已做V7/V8修正映射
 *            ASRPRO → STM32:  "C1\r\n"  (语音"打开左转灯")
 *                             "C2\r\n"  (语音"关闭左转灯")
 *                             "C3\r\n"  (语音"打开右转灯")
 *                             "C4\r\n"  (语音"关闭右转灯")
 *
 *          UART2: PA2=TX, PA3=RX, 9600-8-N-1
 *          TX via existing UART2_SendString() in hcsr04.c
 ******************************************************************************
 */

#include "voice.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 *  Line buffer for interrupt-driven reception
 *
 *  Same strategy as GPS: ISR fills line buffer, sets ready flag
 *  on '\n'.  Voice thread polls, copies, and parses.
 *  At 9600 baud: ~1 char/ms, commands are 2-3 chars — no risk
 *  of overflow with a 16-byte buffer.
 * ================================================================ */
#define VOICE_LINE_BUF_SIZE   16

static volatile uint8_t  voice_line_buf[VOICE_LINE_BUF_SIZE];
static volatile uint8_t  voice_line_idx   = 0;
static volatile uint8_t  voice_line_ready = 0;

/* ---- Internal buffer for CheckLine copy-out ---- */
static char s_line_copy[VOICE_LINE_BUF_SIZE];


/* ================================================================
 *  Voice_UART_RX_Init — enable RXNE interrupt on USART2
 *
 *  NOTE: USART2 peripheral + GPIO are already initialized by
 *  UART2_Init() in hcsr04.c.  This function only enables the
 *  interrupt and configures NVIC.
 *
 *  Preemption priority 2 (lower than GPS=1, higher than default)
 * ================================================================ */
void Voice_UART_RX_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable RXNE interrupt on USART2 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* NVIC configuration */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/* ================================================================
 *  Voice_UART_IRQHandler — USART2 RXNE interrupt handler
 *
 *  Called from USART2_IRQHandler in stm32f10x_it.c.
 *  Fills line buffer; on '\n', NUL-terminates and sets
 *  voice_line_ready = 1.
 *
 *  WARNING: runs in ISR context — no blocking calls.
 * ================================================================ */
void Voice_UART_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART2);

        /* Line complete (LF received) */
        if (ch == '\n')
        {
            if (voice_line_idx > 0 &&
                voice_line_buf[voice_line_idx - 1] == '\r')
                voice_line_idx--;                       /* strip trailing CR     */
            voice_line_buf[voice_line_idx] = '\0';      /* NUL-terminate         */
            voice_line_idx  = 0;
            voice_line_ready = 1;                       /* signal voice thread   */
        }
        /* Discard CR, skip buffer overflow */
        else if (ch == '\r')
        {
            /* nothing — LF will handle termination */
        }
        else if (voice_line_idx < VOICE_LINE_BUF_SIZE - 1)
        {
            voice_line_buf[voice_line_idx++] = ch;
        }
        else
        {
            /* Buffer overflow — reset, wait for next line */
            voice_line_idx = 0;
        }
    }
}


/* ================================================================
 *  Voice_CheckLine — poll for a complete received line
 *
 *  Returns 1 if a line was ready and copied out, 0 otherwise.
 *  The parsed line is stored internally; caller then uses
 *  Voice_ParseLine() on the internal copy.
 *
 *  Thread-safe: disables RXNE IRQ during copy.
 * ================================================================ */
uint8_t Voice_CheckLine(void)
{
    if (!voice_line_ready)
        return 0;

    /* Critical section: disable RXNE, copy, re-enable */
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    strcpy(s_line_copy, (const char *)voice_line_buf);
    voice_line_ready = 0;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    return 1;
}


/* ================================================================
 *  Voice_ParseLine — parse received command line
 *
 *  Expected format: "Cx" where x = 1..4
 *  Returns voice_cmd_t enum, or VOICE_CMD_NONE on unrecognized.
 * ================================================================ */
voice_cmd_t Voice_ParseLine(const char *line)
{
    if (line == NULL || line[0] != 'C')
        return VOICE_CMD_NONE;

    switch (line[1])
    {
    case '1': return VOICE_CMD_LEFT_ON;     /* C1: 左转灯亮 */
    case '2': return VOICE_CMD_LEFT_OFF;    /* C2: 左转灯灭 */
    case '3': return VOICE_CMD_RIGHT_ON;    /* C3: 右转灯亮 */
    case '4': return VOICE_CMD_RIGHT_OFF;   /* C4: 右转灯灭 */
    default:  return VOICE_CMD_NONE;
    }
}


/* ================================================================
 *  Voice_ParseLineFromISR — parse directly from ISR buffer
 *
 *  Convenience: wraps Voice_ParseLine with the internal copy.
 *  Call Voice_CheckLine() first — if it returns 1, then call
 *  this to get the parsed command.
 * ================================================================ */
voice_cmd_t Voice_GetCommand(void)
{
    return Voice_ParseLine(s_line_copy);
}


/* ================================================================
 *  Voice_SendCmd — send a protocol string via UART2
 *
 *  Sends the command via UART2_SendString (which appends "\r\n").
 *  Example: Voice_SendCmd("D")    → sends "D\r\n"
 *
 *  NOTE: ASRPRO 模块的 V7/V8 语音映射与实际相反:
 *        V7 → ASRPRO 播报"右转", V8 → ASRPRO 播报"左转"
 *        因此此处做内部修正: 请求V7实际发V8, 请求V8实际发V7
 *
 *  Blocking TX — takes ~5ms per call at 9600 baud.
 *  Called ONLY from voice thread (single writer = no race).
 * ================================================================ */
void Voice_SendCmd(const char *cmd)
{
    extern void UART2_SendString(const char *str);

    /* ASRPRO V7/V8 映射修正: 模块实际语音与命令相反 */
    if (strcmp(cmd, "V7") == 0)
        UART2_SendString("V8");     /* 调用方请求"左转" → 实际发V8 */
    else if (strcmp(cmd, "V8") == 0)
        UART2_SendString("V7");     /* 调用方请求"右转" → 实际发V7 */
    else
        UART2_SendString(cmd);      /* "D"等其它命令原样发送 */
}
