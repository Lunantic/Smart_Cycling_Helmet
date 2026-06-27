/**
 ******************************************************************************
 * @file    gps.h
 * @brief   GPS NEO-6M driver for STM32F103C8T6
 *          Hardware USART1: PA9(TX), PA10(RX) @ 9600 baud
 *
 *          NMEA sentences parsed:
 *            $GPRMC — lat/lon, speed, course, time, validity
 *            $GPGGA — altitude, satellites, fix quality
 *
 *          NOTE: NEO-6M does NOT have a temperature sensor.
 *                The "temp" field in output is reserved for future
 *                integration (e.g. MPU6050 on-chip temperature,
 *                external DS18B20, etc.)
 ******************************************************************************
 */

#ifndef __GPS_H
#define __GPS_H

#include "stm32f10x.h"

/* ---- GPS data structure ---- */
typedef struct {
    /* Position */
    double  latitude;        /* Decimal degrees, + = N, - = S          */
    double  longitude;       /* Decimal degrees, + = E, - = W          */
    float   altitude;        /* Meters above MSL (from GPGGA)          */

    /* Motion */
    float   speed;           /* Speed over ground, knots (from GPRMC)  */
    float   course;          /* Course over ground, degrees (GPRMC)    */

    /* Fix info */
    uint8_t satellites;      /* Satellites in use (from GPGGA)         */
    uint8_t fix_quality;     /* 0=invalid, 1=GPS fix, 2=DGPS (GPGGA)  */
    uint8_t valid;           /* 1=GPRMC status 'A', 0='V' / no fix    */

    /* UTC Time */
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint8_t year;            /* 2-digit (e.g. 26 = 2026)               */
} gps_data_t;

/* ---- Raw NMEA fields (internal, exposed for debug) ---- */
typedef struct {
    char talker_id[3];       /* "GP" = GPS, "GN" = multi-constellation */
    char sentence_id[4];     /* "RMC", "GGA", etc.                     */
    uint8_t checksum_ok;     /* 1 = checksum matched                    */
} gps_raw_info_t;

/* ---- Global GPS state — updated by parser, read by gps_thread ---- */
extern volatile gps_data_t g_gps_data;
extern volatile uint8_t    g_gps_updated;   /* Toggle on each new fix  */

/* ---- Function prototypes ---- */
void   GPS_UART_Init(void);                   /* USART1 9600-8-N-1, RXNE interrupt */
void   GPS_UART_IRQHandler(void);             /* Call from USART1_IRQHandler in stm32f10x_it.c */
void   GPS_ProcessLine(const char *line);     /* Parse one complete NMEA sentence   */
uint8_t GPS_CheckLine(void);                  /* Poll + process a buffered line (1=processed) */
double GPS_NMEA2Decimal(double nmea, char quadrant);
void   GPS_Set5Hz(void);                      /* Send UBX-CFG-RATE to set 5Hz update (200ms) */

#endif /* __GPS_H */
