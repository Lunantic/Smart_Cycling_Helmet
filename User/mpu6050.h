/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   MPU6050 6-axis sensor driver for STM32F103C8T6
 *          Hardware I2C1: PB6(SCL), PB7(SDA)
 *
 *          Fall: impact + angular-velocity combo detection
 *          Turn: gyroscope Z-axis with lockout debounce
 ******************************************************************************
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/* MPU6050 I2C slave address (7-bit: 0x68, shifted: 0xD0) */
#define MPU6050_ADDR        0xD0

/* MPU6050 registers */
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_PWR_MGMT_2   0x6C
#define MPU6050_WHO_AM_I     0x75

/* ================================================================
 *  Detection thresholds — tuned to eliminate false positives
 * ================================================================
 *  The sensor thread samples @ 100Hz (10ms period).
 *  Debounce counters use *_MS/10 as the count-to-beat, so timing
 *  must match the 10ms sample rate.
 *
 *  FALL — two parallel detection paths (either triggers):
 *
 *    Path A — Combo (side-swipe / roll-over crash):
 *      |total_accel| > FALL_ACCEL_THRESHOLD   AND
 *      |gyro_z|     > FALL_GYRO_THRESHOLD
 *      both sustained for FALL_COMBO_CONFIRM_MS.
 *      Potholes fail (no rotation), head-checks fail (no impact).
 *
 *    Path B — High-G bypass (direct head-on impact / hard knock):
 *      |total_accel| > FALL_HIGHG_THRESHOLD  alone
 *      sustained for FALL_HIGHG_CONFIRM_MS.
 *      No gyro check — a severe enough straight-line impact is a
 *      crash regardless of rotation (e.g. car rear-ends cyclist).
 *      Also used for bench-top knock testing.
 *
 *  TURN:
 *    |gyro_z| > TURN_GYRO_THRESHOLD sustained for TURN_HOLD_MS.
 *    After a turn ends, opposite-direction detection is suppressed
 *    for TURN_LOCKOUT_MS to ignore gyro mechanical rebound.
 */
#define FALL_ACCEL_THRESHOLD          3.50f   /* g, combo impact zone           */
#define FALL_GYRO_THRESHOLD           150.0f  /* °/s, crash-level rotation     */
#define FALL_COMBO_CONFIRM_MS         40      /* ms, both must be sustained     */
#define FALL_HIGHG_THRESHOLD          2.50f   /* g, severe impact (no gyro req) */
#define FALL_HIGHG_CONFIRM_MS         20      /* ms, brief confirmation         */
#define FALL_HOLD_MS                  1000    /* ms, hold alarm (5 blinks × 200ms) */

#define TURN_GYRO_THRESHOLD           20.0f   /* °/s, real turn threshold */
#define TURN_HOLD_MS                  200     /* ms, debounce for turn    */
#define TURN_LOCKOUT_MS               500     /* ms, reject gyro rebound  */

/* System status enum */
typedef enum {
    BIKE_STATUS_NORMAL = 0,
    BIKE_STATUS_FALL,
    BIKE_STATUS_LEFT_TURN,
    BIKE_STATUS_RIGHT_TURN,
} bike_status_t;

/* Raw sensor data */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_t;

/* Physical values */
typedef struct {
    float accel_x_g;      /* g */
    float accel_y_g;
    float accel_z_g;
    float accel_mag_g;    /* total magnitude in g */
    float gyro_z_dps;     /* degrees per second */
} mpu6050_data_t;

/* Global state (volatile - shared between threads) */
extern volatile bike_status_t g_bike_status;
extern volatile float         g_accel_mag;
extern volatile float         g_gyro_z;
extern volatile int16_t       g_accel_raw[3];

/* Function prototypes */
void MPU6050_I2C_Init(void);
int  MPU6050_Init(void);
uint8_t MPU6050_ReadID(void);
int  MPU6050_ReadRaw(mpu6050_raw_t *raw);
void MPU6050_Convert(mpu6050_raw_t *raw, mpu6050_data_t *data);
void MPU6050_Update(void);
bike_status_t MPU6050_DetectStatus(mpu6050_data_t *data);

#endif /* __MPU6050_H */
