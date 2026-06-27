/**
 ******************************************************************************
 * @file    bsp_led.c
 * @brief   委托 ws2812_init() 完成所有 LED 初始化
 *
 *          PA8  — 左灯带数据线 (单线可寻址协议, 10 LEDs)
 *          PA11 — 右灯带数据线 (单线可寻址协议, 10 LEDs)
 *          PB5  — 系统心跳指示灯 (GPIO 推挽输出)
 ******************************************************************************
 */

#include "bsp_led.h"
#include "ws2812.h"

void LED_GPIO_Config(void)
{
    /*
     * ws2812_init() 初始化 PA8/PA11, 清空灯带缓冲.
     * 内部流程:
     *   1. 直接寄存器操作, 最快拉低引脚 (避免上电浮空)
     *   2. 清零缓冲区
     *   3. 发送全零数据 → RESET → 三次 CLEAR 确保全灭
     */

    ws2812_init();

    /*
     * PB5 心跳 LED 独立初始化 (普通 GPIO, 不受 WS2812 影响)
     */
    {
        GPIO_InitTypeDef s;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        s.GPIO_Pin   = GPIO_Pin_5;
        s.GPIO_Mode  = GPIO_Mode_Out_PP;
        s.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &s);
        GPIO_SetBits(GPIOB, GPIO_Pin_5);   /* 初始关闭 (高电平 = LED灭) */
    }
}
