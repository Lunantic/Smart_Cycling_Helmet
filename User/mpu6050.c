/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 driver implementation
 *          Hardware I2C1 on PB6(SCL), PB7(SDA)
 *          Impact + angular-velocity combo fall detection
 *          + turn detection with lockout
 *
 *          I2C timeout added — all bus operations return error if
 *          MPU6050 is not connected, preventing thread starvation.
 ******************************************************************************
 */

#include "mpu6050.h"
#include <rtthread.h>
#include <math.h>

/* ---- Global state ---- */
volatile bike_status_t g_bike_status = BIKE_STATUS_NORMAL;
volatile float         g_accel_mag   = 1.0f;
volatile float         g_gyro_z      = 0.0f;
volatile int16_t       g_accel_raw[3] = {0, 0, 0};

/* ---- I2C timeout helper ---- */
#define I2C_TIMEOUT  5000    /* ~500us @ 72MHz (100kHz I2C byte = ~90us, 5x margin) */

/**
 * @brief  I2C bus recovery — clock SCL manually to release a stuck SDA.
 *
 *         When MPU6050 slave holds SDA low (out-of-sync / power glitch),
 *         simply re-enabling the I2C peripheral does NOT fix the bus.
 *         The standard recovery: pulse SCL up to 9 times until SDA goes
 *         high, then send a STOP condition.  After this, the I2C
 *         peripheral can be re-initialised cleanly.
 *
 *         PB6 = SCL (I2C1), PB7 = SDA (I2C1).  This function temporarily
 *         reassigns them as GPIO open-drain to perform the recovery.
 */
static void I2C1_BusRecovery(void)
{
    GPIO_InitTypeDef s;
    volatile uint32_t d;
    uint8_t i;

    /* 1. Disable I2C1 peripheral, release its grip on the pins */
    I2C_Cmd(I2C1, DISABLE);
    for (d = 0; d < 100; d++) __NOP();

    /* 2. Take SCL + SDA as GPIO open-drain */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    s.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;   /* PB6=SCL, PB7=SDA */
    s.GPIO_Mode  = GPIO_Mode_Out_OD;
    s.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &s);

    /* Set both high initially */
    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    for (d = 0; d < 100; d++) __NOP();

    /* 3. Clock SCL up to 9 times until SDA is released by slave */
    for (i = 0; i < 9; i++)
    {
        /* Check if slave has released SDA */
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_RESET)
            break;   /* SDA high — bus free! */

        /* Pulse SCL low → high (~10us period @ 100kHz) */
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);   /* SCL low  */
        for (d = 0; d < 50; d++) __NOP();    /* ~5us     */
        GPIO_SetBits(GPIOB, GPIO_Pin_6);     /* SCL high */
        for (d = 0; d < 50; d++) __NOP();    /* ~5us     */
    }

    /* 4. Generate STOP condition: SDA low → SCL high → SDA high */
    GPIO_ResetBits(GPIOB, GPIO_Pin_7);       /* SDA low  */
    for (d = 0; d < 50; d++) __NOP();
    GPIO_SetBits(GPIOB, GPIO_Pin_6);         /* SCL high */
    for (d = 0; d < 100; d++) __NOP();
    GPIO_SetBits(GPIOB, GPIO_Pin_7);         /* SDA high (STOP) */
    for (d = 0; d < 100; d++) __NOP();

    /* 5. Give bus time to settle, then re-init I2C hardware */
    MPU6050_I2C_Init();
}

/* Soft-reset I2C1 peripheral only (lightweight — used when bus is OK) */
static void I2C1_Reset(void)
{
    volatile uint32_t d;
    I2C_Cmd(I2C1, DISABLE);
    for (d = 0; d < 1000; d++) __NOP();
    I2C_Cmd(I2C1, ENABLE);
}

/* ---- Internal state for fall detection (two parallel paths) ---- */
typedef enum {
    FALL_STATE_NORMAL = 0,
    FALL_STATE_COMBO,          /* impact + rotation both triggered (Path A) */
} fall_state_t;

static fall_state_t s_fall_state     = FALL_STATE_NORMAL;
static uint8_t      s_combo_count    = 0;
static uint8_t      s_highg_count    = 0;
static uint16_t     s_fall_hold      = 0;  /* remaining hold ticks after fall */

/* ---- Internal state for turn detection ---- */
static uint8_t  s_turn_debounce  = 0;
static uint8_t  s_turn_lockout   = 0;  /* quiet-sample count after turn ends */
static int8_t   s_turn_direction = 0;  /* +1=right, -1=left, 0=none        */

/* ---- Track consecutive I2C failures for auto-recovery ---- */
static uint8_t  s_i2c_fail_count = 0;
#define I2C_FAIL_RESET_THRESHOLD  3   /* 3 × 10ms = 30ms → full bus recovery */

/* ---- Track zero-data samples (sensor alive on I2C but returning all 0) ---- */
static uint8_t  s_sensor_dead_count = 0;
#define SENSOR_DEAD_THRESHOLD      10  /* 10 × 10ms = 100ms → re-init sensor  */


/* ================================================================
 *  Hardware I2C1 low-level
 * ================================================================ */

/**
 * @brief  Init hardware I2C1: PB6=SCL, PB7=SDA, 100kHz
 */
void MPU6050_I2C_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef   I2C_InitStructure;

    /* Enable clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* PB6=I2C1_SCL, PB7=I2C1_SDA: Alternate function open-drain */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C1: 100kHz standard mode */
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}


/**
 * @brief  Write one byte to an MPU6050 register
 *         Returns 0 on success, -1 on timeout.
 */
static int MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    volatile uint32_t timeout;

    /* Generate START */
    timeout = I2C_TIMEOUT;
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    { if (--timeout == 0) goto err; }

    /* Send device address (write) */
    timeout = I2C_TIMEOUT;
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    { if (--timeout == 0) goto err; }

    /* Send register address */
    timeout = I2C_TIMEOUT;
    I2C_SendData(I2C1, reg);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    { if (--timeout == 0) goto err; }

    /* Send data byte */
    timeout = I2C_TIMEOUT;
    I2C_SendData(I2C1, data);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    { if (--timeout == 0) goto err; }

    /* Generate STOP */
    I2C_GenerateSTOP(I2C1, ENABLE);
    s_i2c_fail_count = 0;
    return 0;

err:
    I2C1_Reset();
    return -1;
}


/**
 * @brief  Read multiple bytes from MPU6050 starting at a register
 *         Returns 0 on success, -1 on timeout.
 */
static int MPU6050_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    volatile uint32_t timeout;

    /* Phase 1: Write register address (no STOP) */
    timeout = I2C_TIMEOUT;
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    { if (--timeout == 0) goto err; }

    timeout = I2C_TIMEOUT;
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    { if (--timeout == 0) goto err; }

    timeout = I2C_TIMEOUT;
    I2C_SendData(I2C1, reg);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    { if (--timeout == 0) goto err; }

    /* Phase 2: Repeated START + read */
    timeout = I2C_TIMEOUT;
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    { if (--timeout == 0) goto err; }

    timeout = I2C_TIMEOUT;
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    { if (--timeout == 0) goto err; }

    /* Read bytes: ACK for all except last byte → NACK */
    for (i = 0; i < len; i++)
    {
        if (i == len - 1)
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
        { if (--timeout == 0) goto err; }
        buf[i] = I2C_ReceiveData(I2C1);
    }
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* STOP */
    I2C_GenerateSTOP(I2C1, ENABLE);
    s_i2c_fail_count = 0;
    return 0;

err:
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C1_Reset();
    return -1;
}


/* ================================================================
 *  MPU6050 high-level API
 * ================================================================ */

/**
 * @brief  Read WHO_AM_I register (should return 0x68)
 *         Returns 0 on error (timeout or I2C failure).
 */
uint8_t MPU6050_ReadID(void)
{
    uint8_t id = 0;
    if (MPU6050_ReadMulti(MPU6050_WHO_AM_I, &id, 1) != 0)
        return 0;   /* timeout */
    return id;
}


/**
 * @brief  Initialize MPU6050:
 *         - Wake up from sleep
 *         - ±16g accelerometer range (2048 LSB/g) — crash-impact detection
 *         - ±250°/s gyroscope range (131 LSB/°/s)
 *         - 1kHz sample rate
 *         Returns 0 on success, -1 on failure (no sensor / I2C error).
 */
int MPU6050_Init(void)
{
    uint8_t id;
    uint8_t retry = 5;

    /* Init I2C hardware */
    MPU6050_I2C_Init();

    /* Verify connection */
    while (retry--)
    {
        id = MPU6050_ReadID();
        if (id == 0x68)
            break;
        rt_thread_mdelay(10);
    }
    if (id != 0x68)
        return -1;

    /* Wake up (clear sleep bit) */
    if (MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00) != 0) return -1;
    rt_thread_mdelay(10);

    /* Sample rate divider: 1kHz / (1+7) = 125Hz internal */
    if (MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07) != 0) return -1;

    /* Config: DLPF off (bandwidth 260Hz gyro, 1kHz accel) */
    if (MPU6050_WriteReg(MPU6050_CONFIG, 0x00) != 0) return -1;

    /* Gyro config: ±250°/s */
    if (MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00) != 0) return -1;

    /* Accel config: ±16g (2048 LSB/g) — needed for crash-impact detection */
    if (MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18) != 0) return -1;

    return 0;
}


/**
 * @brief  Read all raw sensor data (14 bytes from ACCEL_XOUT_H)
 *         Returns 0 on success, -1 on I2C timeout.
 */
int MPU6050_ReadRaw(mpu6050_raw_t *raw)
{
    uint8_t buf[14];

    if (MPU6050_ReadMulti(MPU6050_ACCEL_XOUT_H, buf, 14) != 0)
    {
        s_i2c_fail_count++;
        return -1;
    }

    raw->accel_x = (int16_t)(buf[0]  << 8 | buf[1]);
    raw->accel_y = (int16_t)(buf[2]  << 8 | buf[3]);
    raw->accel_z = (int16_t)(buf[4]  << 8 | buf[5]);
    raw->temp    = (int16_t)(buf[6]  << 8 | buf[7]);
    raw->gyro_x  = (int16_t)(buf[8]  << 8 | buf[9]);
    raw->gyro_y  = (int16_t)(buf[10] << 8 | buf[11]);
    raw->gyro_z  = (int16_t)(buf[12] << 8 | buf[13]);
    return 0;
}


/**
 * @brief  Convert raw data to physical units
 */
void MPU6050_Convert(mpu6050_raw_t *raw, mpu6050_data_t *data)
{
    /* ±16g → 2048 LSB/g */
    data->accel_x_g = raw->accel_x / 2048.0f;
    data->accel_y_g = raw->accel_y / 2048.0f;
    data->accel_z_g = raw->accel_z / 2048.0f;

    /* Total acceleration magnitude */
    data->accel_mag_g = sqrtf(data->accel_x_g * data->accel_x_g +
                              data->accel_y_g * data->accel_y_g +
                              data->accel_z_g * data->accel_z_g);

    /* ±250°/s → 131 LSB/°/s */
    data->gyro_z_dps = raw->gyro_z / 131.0f;
}


/**
 * @brief  Detect fall and turn based on sensor data.
 *
 *   FALL — Two parallel detection paths (either triggers):
 *     Path A (COMBO): |accel| > FALL_ACCEL_THRESHOLD  AND
 *                     |gyro_z| > FALL_GYRO_THRESHOLD
 *                     both sustained for FALL_COMBO_CONFIRM_MS.
 *     Path B (HIGHG): |accel| > FALL_HIGHG_THRESHOLD  alone
 *                     sustained for FALL_HIGHG_CONFIRM_MS.
 *                     No gyro check — for severe direct impacts and
 *                     bench-top knock testing.
 *     After confirmation on either path, alarm holds for FALL_HOLD_MS.
 *
 *   TURN — Consecutive-sample counting with direction-change lockout:
 *     |gyro_z| > THRESHOLD sustained ≥ TURN_HOLD_MS confirms a turn.
 *     After a turn ends, opposite detection is suppressed for
 *     TURN_LOCKOUT_MS to ignore gyro mechanical rebound.
 *     Turn detection is suppressed during fall-detection phases.
 */
bike_status_t MPU6050_DetectStatus(mpu6050_data_t *data)
{
    uint8_t  combo_need     = (uint8_t)(FALL_COMBO_CONFIRM_MS / 10);
    uint8_t  highg_need     = (uint8_t)(FALL_HIGHG_CONFIRM_MS  / 10);
    uint16_t fall_hold_need = (uint16_t)(FALL_HOLD_MS           / 10);
    uint8_t  turn_need      = (uint8_t)(TURN_HOLD_MS            / 10);
    uint8_t  lockout_need   = (uint8_t)(TURN_LOCKOUT_MS         / 10);
    float    gyro_abs;

    /* ---- Fall hold: keep alarm active for FALL_HOLD_MS after confirmed fall ---- */
    if (s_fall_hold > 0)
    {
        s_fall_hold--;
        s_fall_state   = FALL_STATE_NORMAL;
        s_combo_count  = 0;
        s_highg_count  = 0;
        s_turn_debounce  = 0;
        s_turn_direction = 0;
        s_turn_lockout   = 0;
        return BIKE_STATUS_FALL;
    }

    gyro_abs = data->gyro_z_dps;
    if (gyro_abs < 0.0f) gyro_abs = -gyro_abs;

    /* ================================================================
     *  Fall detection — Path A (combo) + Path B (high-G bypass)
     *
     *  Path B runs independently from NORMAL state, even while
     *  Path A is in COMBO state.  A severe-enough impact always wins.
     * ================================================================ */

    /* ---- Path B: High-G bypass (check EVERY call, regardless of state) ---- */
    if (data->accel_mag_g > FALL_HIGHG_THRESHOLD)
    {
        s_highg_count++;
        if (s_highg_count >= highg_need)
        {
            /* ======== FALL CONFIRMED (Path B — high-G) ======== */
            s_fall_hold      = fall_hold_need;
            s_fall_state     = FALL_STATE_NORMAL;
            s_combo_count    = 0;
            s_highg_count    = 0;
            s_turn_debounce  = 0;
            s_turn_direction = 0;
            s_turn_lockout   = 0;
            return BIKE_STATUS_FALL;
        }
    }
    else
    {
        s_highg_count = 0;   /* impact dropped below high-G threshold */
    }

    /* ---- Path A: Combo state machine (accel + gyro) ---- */
    switch (s_fall_state)
    {
    case FALL_STATE_NORMAL:
        if (data->accel_mag_g > FALL_ACCEL_THRESHOLD &&
            gyro_abs > FALL_GYRO_THRESHOLD)
        {
            s_fall_state  = FALL_STATE_COMBO;
            s_combo_count = 1;
        }
        break;

    case FALL_STATE_COMBO:
        if (data->accel_mag_g > FALL_ACCEL_THRESHOLD &&
            gyro_abs > FALL_GYRO_THRESHOLD)
        {
            s_combo_count++;
            if (s_combo_count >= combo_need)
            {
                /* ======== FALL CONFIRMED (Path A — combo) ======== */
                s_fall_hold      = fall_hold_need;
                s_fall_state     = FALL_STATE_NORMAL;
                s_combo_count    = 0;
                s_highg_count    = 0;
                s_turn_debounce  = 0;
                s_turn_direction = 0;
                s_turn_lockout   = 0;
                return BIKE_STATUS_FALL;
            }
        }
        else
        {
            /* Combo broken — false alarm, reset */
            s_fall_state  = FALL_STATE_NORMAL;
            s_combo_count = 0;
        }
        break;
    }

    /* ================================================================
     *  Turn detection (suppressed during fall-detection phases)
     * ================================================================ */
    if (s_fall_state == FALL_STATE_NORMAL)
    {
        if (data->gyro_z_dps > TURN_GYRO_THRESHOLD)
        {
            if (s_turn_lockout == 0)
            {
                if (s_turn_direction != 1) { s_turn_debounce = 0; s_turn_direction = 1; }
                s_turn_debounce++;
                if (s_turn_debounce >= turn_need)
                {
                    s_turn_lockout = lockout_need;
                    return BIKE_STATUS_RIGHT_TURN;
                }
            }
        }
        else if (data->gyro_z_dps < -TURN_GYRO_THRESHOLD)
        {
            if (s_turn_lockout == 0)
            {
                if (s_turn_direction != -1) { s_turn_debounce = 0; s_turn_direction = -1; }
                s_turn_debounce++;
                if (s_turn_debounce >= turn_need)
                {
                    s_turn_lockout = lockout_need;
                    return BIKE_STATUS_LEFT_TURN;
                }
            }
        }
        else
        {
            /* Gyro in quiet zone: count down lockout, reset debounce */
            s_turn_debounce  = 0;
            s_turn_direction = 0;
            if (s_turn_lockout > 0)
                s_turn_lockout--;
        }
    }
    else
    {
        /* During fall-detection phases, suppress turn detection */
        s_turn_debounce  = 0;
        s_turn_direction = 0;
        if (s_turn_lockout > 0)
            s_turn_lockout--;
    }

    return BIKE_STATUS_NORMAL;
}


/**
 * @brief  Full update cycle: read sensor → convert → detect → update globals
 *         Called every 10ms from sensor thread.
 *
 *         Recovery strategies (two independent mechanisms):
 *         1. I2C timeout → bus recovery (bit-bang SCL) after N consecutive fails
 *         2. Sensor returns all-zero → full MPU6050 re-init after N consecutive
 *            zero samples (handles sleep mode / clock failure where I2C still ACKs)
 */
void MPU6050_Update(void)
{
    mpu6050_raw_t  raw;
    mpu6050_data_t data;

    if (MPU6050_ReadRaw(&raw) != 0)
    {
        /* ---- I2C read failed — bus may be stuck ---- */
        if (s_i2c_fail_count >= I2C_FAIL_RESET_THRESHOLD)
        {
            I2C1_BusRecovery();
            s_i2c_fail_count       = 0;
            s_sensor_dead_count    = 0;
        }
        return;
    }

    /* ---- I2C OK, but check for "zombie" sensor (all registers = 0) ---- */
    if (raw.accel_x == 0 && raw.accel_y == 0 && raw.accel_z == 0 &&
        raw.gyro_x  == 0 && raw.gyro_y  == 0 && raw.gyro_z  == 0)
    {
        s_sensor_dead_count++;
        if (s_sensor_dead_count >= SENSOR_DEAD_THRESHOLD)
        {
            /* Sensor responds to I2C but all data is zero — likely
             * in sleep mode or internal clock stopped.  Full re-init. */
            MPU6050_Init();
            s_sensor_dead_count = 0;
            s_i2c_fail_count    = 0;
        }
        /* Don't update globals with zero data */
        return;
    }
    s_sensor_dead_count = 0;   /* data OK, reset dead counter */

    MPU6050_Convert(&raw, &data);

    /* Update globals for UI thread */
    g_accel_raw[0] = raw.accel_x;
    g_accel_raw[1] = raw.accel_y;
    g_accel_raw[2] = raw.accel_z;
    g_accel_mag    = data.accel_mag_g;
    g_gyro_z       = data.gyro_z_dps;
    g_bike_status  = MPU6050_DetectStatus(&data);
}
