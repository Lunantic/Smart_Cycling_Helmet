/**
 ******************************************************************************
 * @file    hcsr04.c
 * @brief   HC-SR04 ultrasonic sensor driver — HW input capture + semaphore
 *          TIM2 CH1 (PA0) dual-edge capture with digital filter (0xF)
 *          RT-Thread semaphore replaces busy-wait polling.
 *
 *          TIM2 runs at 1MHz (1us per tick).  Maximum range 450cm gives
 *          ~26ms echo round-trip.  Semaphore timeout = 100ms.
 *
 *          The STM32 digital filter (ICFilter=0xF) suppresses breadboard
 *          parasitic coupling at the hardware level — no busy-waiting
 *          needed.  The thread sleeps on a semaphore during echo wait;
 *          all other RT-Thread threads run freely.
 *
 *          UART2 on PA2(TX) / PA3(RX) @ 9600 (ASRPRO voice module)
 ******************************************************************************
 */

#include "hcsr04.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include <rtthread.h>

/* ---- Digital filter: 0xF = 8 consecutive samples must agree @ f_DTS ----
 *    1MHz DTS → 8us glitch rejection.  Eliminates breadboard crosstalk
 *    without any busy-waiting or software debounce.                      */
#define HCSR04_IC_FILTER    0x0F

/* ---- Ringing / crosstalk reject threshold (in timer ticks = us) ---- */
#define RINGING_MIN_US      120     /* pulses < 120us = noise              */

/* ---- TIM2 overflow limit for timeout ---- */
#define TIM2_OVF_LIMIT      1       /* 1 × 65.5ms > real max echo 26ms    */

/* ================================================================
 *  Internal state
 * ================================================================ */
static rt_sem_t           s_hcsr04_sem      = RT_NULL;
static volatile uint32_t  s_capture_width;    /* pulse width in us         */
static volatile uint8_t   s_capture_valid;    /* 1 = valid pulse captured  */
static volatile uint8_t   s_capture_overflow; /* TIM2 overflow counter     */
static volatile uint8_t   s_edge_phase;       /* 0=wait_rise, 1=wait_fall */
static volatile uint16_t  s_rise_cnt;         /* CCR1 value at rising edge */


/* ================================================================
 *  GPIO init: PA1=TRIG (push-pull), PA0=ECHO (TIM2 CH1 AF)
 * ================================================================ */
static void HCSR04_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA1 = TRIG: push-pull output, init LOW */
    GPIO_InitStructure.GPIO_Pin   = HCSR04_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HCSR04_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    /* PA0 = ECHO: TIM2 CH1 input capture + internal pull-down
     *   Pull-down keeps pin LOW when HC-SR04 is disconnected,
     *   preventing noise-induced spurious captures.           */
    GPIO_InitStructure.GPIO_Pin   = HCSR04_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_Init(HCSR04_ECHO_PORT, &GPIO_InitStructure);
}


/* ================================================================
 *  TIM2 init: 1MHz free-running + CH1 input capture
 *  Digital filter 0xF suppresses breadboard crosstalk in hardware.
 * ================================================================ */
static void HCSR04_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(HCSR04_TIM_CLK, ENABLE);

    /* ---- Time base: 1us per tick, free-running ---- */
    TIM_TimeBaseStructure.TIM_Period        = HCSR04_TIM_ARR;      /* 0xFFFF */
    TIM_TimeBaseStructure.TIM_Prescaler     = HCSR04_TIM_PSC;      /* 71    */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(HCSR04_TIM, &TIM_TimeBaseStructure);

    /* ---- CH1 input capture: rising edge first, digital filter ---- */
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = HCSR04_IC_FILTER;       /* 8-sample glitch reject */
    TIM_ICInit(HCSR04_TIM, &TIM_ICInitStructure);

    /* ---- Enable CC1 + Update interrupts ---- */
    TIM_ITConfig(HCSR04_TIM, TIM_IT_CC1 | TIM_IT_Update, ENABLE);

    /* ---- NVIC: TIM2 interrupt at preemption priority 1 ---- */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* ---- Start free-running counter ---- */
    TIM_Cmd(HCSR04_TIM, ENABLE);
}


/* ================================================================
 *  Public: full init
 * ================================================================ */
void HCSR04_Init(void)
{
    HCSR04_GPIO_Init();
    HCSR04_TIM_Init();

    /* Create binary semaphore (initial value 0) for HW→thread sync */
    if (s_hcsr04_sem == RT_NULL)
        s_hcsr04_sem = rt_sem_create("hcsr04", 0, RT_IPC_FLAG_FIFO);
}


/* ================================================================
 *  Trigger: send 10us high pulse on PA1, reset capture state
 * ================================================================ */
void HCSR04_Trigger(void)
{
    /* Drain any stale semaphore count from a previous overflow/timeout */
    if (s_hcsr04_sem != RT_NULL)
    {
        while (rt_sem_take(s_hcsr04_sem, 0) == RT_EOK);
    }

    /* Reset state for new measurement */
    s_capture_width    = 0;
    s_capture_valid    = 0;
    s_capture_overflow = 0;
    s_edge_phase       = 0;   /* wait for rising edge first */
    s_rise_cnt         = 0;

    /* Reset counter to 0 */
    TIM_SetCounter(HCSR04_TIM, 0);

    /* Ensure CH1 is configured for rising edge */
    {
        TIM_ICInitTypeDef ic;
        ic.TIM_Channel     = TIM_Channel_1;
        ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
        ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
        ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        ic.TIM_ICFilter    = HCSR04_IC_FILTER;
        TIM_ICInit(HCSR04_TIM, &ic);
    }

    /* Clear any stale interrupt flags */
    TIM_ClearITPendingBit(HCSR04_TIM, TIM_IT_CC1 | TIM_IT_Update);

    /* Fire 10us trigger pulse */
    GPIO_SetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    {
        volatile uint32_t i;
        for (i = 0; i < 180; i++) __NOP();   /* ~10us @ 72MHz */
    }
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
}


/* ================================================================
 *  GetDistance — wait on semaphore (thread sleeps — no busy-wait!)
 *
 *  Called from ultrasonic_thread_entry every 500ms.
 *  During the 100ms semaphore wait, the thread is SUSPENDED —
 *  all other RT-Thread threads get full CPU time.
 * ================================================================ */
hcsr04_status_t HCSR04_GetDistance(float *distance_cm)
{
    rt_err_t  sem_ret;
    uint32_t  width;

    if (s_hcsr04_sem == RT_NULL)
        return HCSR04_TIMEOUT;

    /* ---- 1. Fire trigger, arm capture ---- */
    HCSR04_Trigger();

    /* ---- 2. Sleep on semaphore (100ms timeout) — CPU released! ---- */
    sem_ret = rt_sem_take(s_hcsr04_sem,
                          rt_tick_from_millisecond(HCSR04_ECHO_TIMEOUT_MS));

    /* ---- 3. Check results ---- */
    if (sem_ret != RT_EOK)
        return HCSR04_TIMEOUT;          /* semaphore timeout — no module? */

    if (s_capture_overflow >= TIM2_OVF_LIMIT)
        return HCSR04_NO_ECHO;          /* TIM2 overflowed — no echo edge */

    if (!s_capture_valid)
        return HCSR04_NO_ECHO;          /* glitch / invalid capture */

    width = s_capture_width;

    /* ---- 4. Filter ringing / crosstalk ---- */
    if (width < RINGING_MIN_US)
        return HCSR04_NO_ECHO;

    /* ---- 5. Convert to distance (us → cm: width/58) ---- */
    *distance_cm = (float)width / 58.0f;

    if (*distance_cm < HCSR04_MIN_DIST_CM)
        return HCSR04_OUT_OF_RANGE_LOW;
    if (*distance_cm > HCSR04_MAX_DIST_CM)
        return HCSR04_OUT_OF_RANGE_HIGH;

    return HCSR04_OK;
}


/* ================================================================
 *  ISR handler — TIM2 input capture (CC1) + Update (overflow)
 *
 *  Edge-capture state machine:
 *    1. Rising edge  → store CCR1 as rise time, switch CH1 to falling
 *    2. Falling edge → compute width = fall - rise, post semaphore
 *
 *  Overflow: after TIM2_OVF_LIMIT overflows, post semaphore so
 *  GetDistance returns HCSR04_NO_ECHO instead of hanging.
 *
 *  Digital filter (ICFilter=0xF) rejects breadboard crosstalk at
 *  the hardware level — the ISR never sees glitches < 8us.
 * ================================================================ */
void HCSR04_IRQHandler(void)
{
    /* ---- Capture/Compare 1 event (edge detected) ---- */
    if (TIM_GetITStatus(HCSR04_TIM, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(HCSR04_TIM, TIM_IT_CC1);

        if (s_edge_phase == 0)
        {
            /* ==== Rising edge: record time, arm falling edge ==== */
            s_rise_cnt = (uint16_t)TIM_GetCapture1(HCSR04_TIM);

            {
                TIM_ICInitTypeDef ic;
                ic.TIM_Channel     = TIM_Channel_1;
                ic.TIM_ICPolarity  = TIM_ICPolarity_Falling;
                ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
                ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
                ic.TIM_ICFilter    = HCSR04_IC_FILTER;
                TIM_ICInit(HCSR04_TIM, &ic);
            }

            s_edge_phase = 1;
        }
        else
        {
            /* ==== Falling edge: compute width, wake thread ==== */
            uint16_t fall  = (uint16_t)TIM_GetCapture1(HCSR04_TIM);
            uint32_t width = (uint32_t)(fall - s_rise_cnt);

            if (width >= RINGING_MIN_US)
            {
                s_capture_width = width;
                s_capture_valid = 1;
            }

            s_edge_phase = 0;

            /* Wake up the ultrasonic thread */
            if (s_hcsr04_sem != RT_NULL)
                rt_sem_release(s_hcsr04_sem);
        }
    }

    /* ---- Update event (TIM2 counter overflow) ---- */
    if (TIM_GetITStatus(HCSR04_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(HCSR04_TIM, TIM_IT_Update);
        s_capture_overflow++;

        if (s_capture_overflow >= TIM2_OVF_LIMIT)
        {
            /* Timeout — wake thread with invalid data */
            s_edge_phase = 0;
            if (s_hcsr04_sem != RT_NULL)
                rt_sem_release(s_hcsr04_sem);
        }
    }
}


/* ================================================================
 *  UART2 init: PA2=TX, PA3=RX, 9600-8-N-1 (ASRPRO voice module)
 * ================================================================ */
void UART2_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 = USART2_TX (alternate function push-pull) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 = USART2_RX (pull-up: prevent noise from unpowered ASRPRO) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}


/* ================================================================
 *  UART2 string send (blocking, ~3ms for "V7\r\n" at 9600 baud)
 * ================================================================ */
void UART2_SendString(const char *str)
{
    while (*str)
    {
        USART_SendData(USART2, (uint16_t)(*str++));
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}
