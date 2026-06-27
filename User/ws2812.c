/**
 ******************************************************************************
 * @file    ws2812.c
 * @brief   双独立灯带位脉冲驱动 — 宏展开版 (无函数调用, 保证纳秒时序)
 *
 *          左灯带: PA8 (DIN), 10 LEDs 级联
 *          右灯带: PA11 (DIN), 10 LEDs 级联
 *
 *          协议: 单极性归零码, 800kHz NZR
 *            T0H ≈ 0.38µs,  T0L ≈ 0.86µs
 *            T1H ≈ 0.76µs,  T1L ≈ 0.53µs
 *            RESET > 80µs LOW (确保 >> 50µs 最小要求)
 *
 *  ⚠ 时序已针对 WS2812B 规范校准 (72MHz):
 *      WS2812B: T0H=250~550ns  T0L=700~1000ns
 *               T1H=650~950ns  T1L=300~600ns
 *      本驱动:   T0H≈389ns     T0L≈861ns
 *               T1H≈764ns     T1L≈528ns
 ******************************************************************************
 */

#include "ws2812.h"

/* ---- 灯带缓冲区 GRB 格式 ---- */
static uint8_t buf_left[WS2812_NUM_LEDS][3];
static uint8_t buf_right[WS2812_NUM_LEDS][3];

/* ---- 炫彩流水灯 8 色调色板 (低亮度 ~25%, 省电) ---- */
const uint32_t rainbow_palette[RAINBOW_COLORS] = {
    0x003C00UL,   /* [0] 红    R=60, G=0,   B=0   */
    0x283C00UL,   /* [1] 橙    R=60, G=40,  B=0   */
    0x3C3C00UL,   /* [2] 黄    R=60, G=60,  B=0   */
    0x3C0000UL,   /* [3] 绿    R=0,  G=60,  B=0   */
    0x3C003CUL,   /* [4] 青    R=0,  G=60,  B=60  */
    0x00003CUL,   /* [5] 蓝    R=0,  G=0,   B=60  */
    0x1E003CUL,   /* [6] 紫    R=0,  G=30,  B=60  */
    0x3C1428UL,   /* [7] 粉    R=20, G=60,  B=40  */
};

/* ================================================================
 *  宏展开位脉冲 — 硬编码引脚, 零函数调用开销
 *
 *  72MHz: 1 cycle ≈ 13.89ns
 *  时序计算 (含BSRR/BRR store开销 ~2c 及循环开销 ~11c):
 *
 *  SEND_0: T0H = 2c(BSRR) + 24 NOPs + 1c ← 27c = 375ns
 *          T0L = 1c(BRR)  + 50 NOPs + 11c(循环) ← 62c = 861ns
 *  SEND_1: T1H = 2c(BSRR) + 52 NOPs + 1c ← 55c = 764ns
 *          T1L = 1c(BRR)  + 26 NOPs + 11c(循环) ← 38c = 528ns
 *
 *  RESET: volatile 循环 1500 次 × ~8~10c ≈ 180µs >> 50µs
 * ================================================================ */

/* ---------- 左灯带 (PA8 = 0x0100) ---------- */
#define SEND_0_LEFT()  do {                     \
    GPIOA->BSRR = 0x0100;    /* PA8 HIGH */     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();             \
    GPIOA->BRR  = 0x0100;    /* PA8 LOW  */     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
} while(0)

#define SEND_1_LEFT()  do {                     \
    GPIOA->BSRR = 0x0100;    /* PA8 HIGH */     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();                             \
    GPIOA->BRR  = 0x0100;    /* PA8 LOW  */     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();                                     \
} while(0)

/* ---------- 右灯带 (PA11 = 0x0800) ---------- */
#define SEND_0_RIGHT() do {                     \
    GPIOA->BSRR = 0x0800;    /* PA11 HIGH */    \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();             \
    GPIOA->BRR  = 0x0800;    /* PA11 LOW  */    \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
} while(0)

#define SEND_1_RIGHT() do {                     \
    GPIOA->BSRR = 0x0800;    /* PA11 HIGH */    \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();                             \
    GPIOA->BRR  = 0x0800;    /* PA11 LOW  */    \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();__NOP();__NOP();__NOP();__NOP();     \
    __NOP();                                     \
} while(0)

/* ================================================================
 *  RESET 码 — >180µs LOW (规范要求 >50µs)
 * ================================================================ */
static void send_reset_left(void)
{
    volatile uint16_t i;
    GPIOA->BRR = 0x0100;
    for (i = 0; i < 1500; i++) {
        __NOP();
    }
}

static void send_reset_right(void)
{
    volatile uint16_t i;
    GPIOA->BRR = 0x0800;
    for (i = 0; i < 1500; i++) {
        __NOP();
    }
}

/* 双灯带同时 RESET — 避免分开发送时另一引脚出现电平窗口 */
static void send_reset_both(void)
{
    volatile uint16_t i;
    GPIOA->BRR = 0x0100 | 0x0800;
    for (i = 0; i < 1500; i++) {
        __NOP();
    }
}

/* ================================================================
 *  底层发送 — 将缓冲区数据位脉冲发出
 * ================================================================ */
static void ws2812_send_left(void)
{
    uint8_t i, bit;
    uint8_t byte_val;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        /* Green */
        byte_val = buf_left[i][0];
        for (bit = 0; bit < 8; bit++) {
            if (byte_val & 0x80) SEND_1_LEFT(); else SEND_0_LEFT();
            byte_val <<= 1;
        }
        /* Red */
        byte_val = buf_left[i][1];
        for (bit = 0; bit < 8; bit++) {
            if (byte_val & 0x80) SEND_1_LEFT(); else SEND_0_LEFT();
            byte_val <<= 1;
        }
        /* Blue */
        byte_val = buf_left[i][2];
        for (bit = 0; bit < 8; bit++) {
            if (byte_val & 0x80) SEND_1_LEFT(); else SEND_0_LEFT();
            byte_val <<= 1;
        }
    }

    send_reset_left();

    if (!primask) __enable_irq();
}

static void ws2812_send_right(void)
{
    uint8_t i, bit;
    uint8_t byte_val;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        /* Green */
        byte_val = buf_right[i][0];
        for (bit = 0; bit < 8; bit++) {
            if (byte_val & 0x80) SEND_1_RIGHT(); else SEND_0_RIGHT();
            byte_val <<= 1;
        }
        /* Red */
        byte_val = buf_right[i][1];
        for (bit = 0; bit < 8; bit++) {
            if (byte_val & 0x80) SEND_1_RIGHT(); else SEND_0_RIGHT();
            byte_val <<= 1;
        }
        /* Blue */
        byte_val = buf_right[i][2];
        for (bit = 0; bit < 8; bit++) {
            if (byte_val & 0x80) SEND_1_RIGHT(); else SEND_0_RIGHT();
            byte_val <<= 1;
        }
    }

    send_reset_right();

    if (!primask) __enable_irq();
}

/* ================================================================
 *  公共 API
 * ================================================================ */

/**
 * @brief  发送单条灯带数据
 */
void ws2812_send_strip(strip_id_t id)
{
    if (id == STRIP_LEFT)
        ws2812_send_left();
    else
        ws2812_send_right();
}

/**
 * @brief  同时发送两条灯带数据 (一次关中断, 减少打断)
 */
void ws2812_send_both(void)
{
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();
    /* 先发左灯带全部数据+RESET, 再发右灯带 */
    {
        uint8_t i, bit;
        uint8_t byte_val;
        for (i = 0; i < WS2812_NUM_LEDS; i++)
        {
            byte_val = buf_left[i][0];
            for (bit = 0; bit < 8; bit++) {
                if (byte_val & 0x80) SEND_1_LEFT(); else SEND_0_LEFT();
                byte_val <<= 1;
            }
            byte_val = buf_left[i][1];
            for (bit = 0; bit < 8; bit++) {
                if (byte_val & 0x80) SEND_1_LEFT(); else SEND_0_LEFT();
                byte_val <<= 1;
            }
            byte_val = buf_left[i][2];
            for (bit = 0; bit < 8; bit++) {
                if (byte_val & 0x80) SEND_1_LEFT(); else SEND_0_LEFT();
                byte_val <<= 1;
            }
        }
    }
    send_reset_left();
    {
        uint8_t i, bit;
        uint8_t byte_val;
        for (i = 0; i < WS2812_NUM_LEDS; i++)
        {
            byte_val = buf_right[i][0];
            for (bit = 0; bit < 8; bit++) {
                if (byte_val & 0x80) SEND_1_RIGHT(); else SEND_0_RIGHT();
                byte_val <<= 1;
            }
            byte_val = buf_right[i][1];
            for (bit = 0; bit < 8; bit++) {
                if (byte_val & 0x80) SEND_1_RIGHT(); else SEND_0_RIGHT();
                byte_val <<= 1;
            }
            byte_val = buf_right[i][2];
            for (bit = 0; bit < 8; bit++) {
                if (byte_val & 0x80) SEND_1_RIGHT(); else SEND_0_RIGHT();
                byte_val <<= 1;
            }
        }
    }
    send_reset_right();
    if (!primask) __enable_irq();
}

/**
 * @brief  整条灯带设相同颜色
 */
void ws2812_set_strip(strip_id_t id, uint32_t color_grb)
{
    uint8_t g = (uint8_t)((color_grb >> 16) & 0xFF);
    uint8_t r = (uint8_t)((color_grb >> 8)  & 0xFF);
    uint8_t b = (uint8_t)( color_grb        & 0xFF);
    uint8_t (*buf)[3] = (id == STRIP_LEFT) ? buf_left : buf_right;
    uint8_t i;

    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        buf[i][0] = g;
        buf[i][1] = r;
        buf[i][2] = b;
    }
}

/**
 * @brief  设置单个灯珠颜色
 * @param  id   灯带选择 (STRIP_LEFT / STRIP_RIGHT)
 * @param  n    灯珠索引 (0 ~ WS2812_NUM_LEDS-1)
 * @param  color_grb  24bit GRB 颜色
 */
void ws2812_set_led(strip_id_t id, uint8_t n, uint32_t color_grb)
{
    uint8_t (*buf)[3];

    if (n >= WS2812_NUM_LEDS) return;

    buf = (id == STRIP_LEFT) ? buf_left : buf_right;

    buf[n][0] = (uint8_t)((color_grb >> 16) & 0xFF);  /* G */
    buf[n][1] = (uint8_t)((color_grb >> 8)  & 0xFF);  /* R */
    buf[n][2] = (uint8_t)( color_grb        & 0xFF);  /* B */
}

/**
 * @brief  两灯带全灭
 */
void ws2812_clear(void)
{
    uint8_t i;
    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        buf_left[i][0]  = 0; buf_left[i][1]  = 0; buf_left[i][2]  = 0;
        buf_right[i][0] = 0; buf_right[i][1] = 0; buf_right[i][2] = 0;
    }
    /* 右灯带优先 — 针对 PA11 上电蓝灯问题 */
    ws2812_send_right();
    ws2812_send_left();
}

/**
 * @brief  在一条灯带上应用彩虹流水效果
 *
 *         每条灯带 10 个灯珠, 使用 8 色调色板循环填充.
 *         step 参数控制流动偏移量, 每帧 +1 产生流动动画.
 *
 *         流动方向: 从左到右 (LED0 最先显示当前色)
 *
 * @param  id    灯带选择
 * @param  step  流动步数 (0~255, 自动循环)
 */
void ws2812_apply_rainbow(strip_id_t id, uint8_t step)
{
    uint8_t i;
    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        uint8_t ci = (i + step) % RAINBOW_COLORS;
        ws2812_set_led(id, i, rainbow_palette[ci]);
    }
    ws2812_send_strip(id);
}

/**
 * @brief  准备彩虹流水 + 另一灯带灭 (只写缓冲, 不发送)
 *
 *         配合 ws2812_send_both() 使用, 实现双灯带原子更新,
 *         消除两次独立发送之间的间隙, 动画更流畅。
 *
 * @param  active  流水灯带
 * @param  other   要熄灭的灯带
 * @param  step    流动步数
 */
void ws2812_prep_rainbow_off(strip_id_t active, strip_id_t other, uint8_t step)
{
    uint8_t i;
    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        uint8_t ci = (i + step) % RAINBOW_COLORS;
        ws2812_set_led(active, i, rainbow_palette[ci]);
    }
    ws2812_set_strip(other, WS2812_COLOR_OFF);
}

/**
 * @brief  报警模式: 双灯带全红 (低亮度, 省电)
 */
void ws2812_apply_alarm(void)
{
    ws2812_set_strip(STRIP_LEFT,  WS2812_COLOR_RED);
    ws2812_set_strip(STRIP_RIGHT, WS2812_COLOR_RED);
    ws2812_send_both();
}

/**
 * @brief  初始化 PA8/PA11
 *
 *         ⚠ 关键: 必须第一时间把 PA8/PA11 驱动为 LOW.
 *         之前代码先清缓冲区再开时钟 — 清缓冲区期间引脚浮空,
 *         WS2812 会把浮空噪声锁存导致上电蓝灯.
 *
 *         新流程: 开时钟 → 直接写 CRH 配输出 → BRR 拉低
 *         → 全程 ~10 CPU 周期. 引脚确定 LOW 后才清缓冲区.
 *
 *         硬件建议: PA8/PA11 各接 10kΩ 下拉电阻到 GND 可根除.
 */
void ws2812_init(void)
{
    uint8_t i;
    volatile uint32_t dly;

    /*
     * Step 1 (CRITICAL): 开时钟 → 直接寄存器操作,
     * 把 PA8/PA11 配为推挽输出 + 拉低.
     * 用直接寄存器写代替 GPIO_Init(), 指令数降到最低,
     * 最大限度缩短引脚浮空窗口 (~10 CPU 周期 ≈ 140ns).
     */
    RCC->APB2ENR |= WS2812_PORT_CLK;

    /* CRH: PA8 在 bit[3:0], PA11 在 bit[15:12]
     * CNF=00(推挽), MODE=11(50MHz) → 0x3 */
    {
        uint32_t crh = GPIOA->CRH;
        crh &= ~((0xFUL << 0) | (0xFUL << 12));
        crh |=  (0x3UL << 0) | (0x3UL << 12);
        GPIOA->CRH = crh;
    }

    /* ODR 复位值 = 0, 再写 BRR 确保两引脚确定 LOW */
    GPIOA->BRR = WS2812_LEFT_PIN | WS2812_RIGHT_PIN;

    /*
     * Step 2: 引脚已确定 LOW, 现在安全地清零缓冲区.
     */
    for (i = 0; i < WS2812_NUM_LEDS; i++)
    {
        buf_left[i][0]  = 0; buf_left[i][1]  = 0; buf_left[i][2]  = 0;
        buf_right[i][0] = 0; buf_right[i][1] = 0; buf_right[i][2] = 0;
    }

    /*
     * Step 3: 立刻对 PA11 右灯带单独发送全零数据.
     *         如果 WS2812 在 MCU 启动期间锁存了蓝灯,
     *         这是最快的覆盖手段 (发生在左灯带任何操作之前).
     */
    ws2812_send_right();
    send_reset_both();

    /*
     * Step 4: 超长 RESET — 拉低 10ms.
     *         足够长的 LOW 电平可强制复位 WS2812 内部状态机.
     */
    GPIOA->BRR = WS2812_LEFT_PIN | WS2812_RIGHT_PIN;
    for (dly = 0; dly < 720000; dly++) __NOP();   /* ~10ms @ 72MHz */

    /* Step 5: 同时 RESET + 三次 CLEAR, 确保所有灯珠全灭 */
    send_reset_both();
    ws2812_clear();

    for (dly = 0; dly < 360000; dly++) __NOP();   /* ~5ms */
    ws2812_clear();

    for (dly = 0; dly < 360000; dly++) __NOP();   /* ~5ms */
    ws2812_clear();
}
