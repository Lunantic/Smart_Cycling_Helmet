/**
 ******************************************************************************
 * @file    gps.c
 * @brief   GPS NEO-6M driver implementation
 *          USART1 RX interrupt → line buffer → NMEA parser
 *
 *          A valid fix requires BOTH:
 *            1. $GPRMC status = 'A'
 *            2. $GPGGA fix quality >= 1
 ******************************************************************************
 */

#include "gps.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 *  Line-buffer for interrupt-driven NMEA reception
 *
 *  Strategy: ISR fills a single line buffer.  When CR/LF arrives,
 *  the line is NUL-terminated and gps_line_ready is set.  The
 *  GPS thread polls gps_line_ready, copies the line, and clears
 *  the flag.  This is simpler and safer than a full ring buffer
 *  for 9600 baud NMEA (~1 char/ms, sentences every 100-1000ms).
 * ================================================================ */
#define GPS_LINE_BUF_SIZE   128

static volatile uint8_t  gps_line_buf[GPS_LINE_BUF_SIZE];
static volatile uint8_t  gps_line_idx  = 0;
static volatile uint8_t  gps_line_ready = 0;

/* ---- Global state ---- */
volatile gps_data_t g_gps_data;
volatile uint8_t    g_gps_updated = 0;

/* ---- Internal parser state ---- */
static gps_data_t   s_parsed;           /* accumulates fields during parse */
static uint8_t      s_gprmc_seen = 0;   /* set when GPRMC fully parsed    */
static uint8_t      s_gpgga_seen = 0;   /* set when GPGGA fully parsed    */


/* ================================================================
 *  USART1 GPIO + peripheral init: PA9=TX, PA10=RX, 9600-8-N-1
 *
 *  USART1 is on APB2 (72 MHz)
 *  Baud = 72MHz / (16 * USARTDIV) → USARTDIV = 72M / (16 * 9600)
 *       = 468.75 → Mantissa=468, Fraction=0.75*16=12=0xC
 * ================================================================ */
void GPS_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* ---- Clock gates ---- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* ---- PA9 = USART1_TX (alternate function push-pull) ---- */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ---- PA10 = USART1_RX (pull-up: prevent noise from unpowered GPS) ---- */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ---- USART1: 9600-8-N-1 ---- */
    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* ---- Enable RXNE interrupt ---- */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* ---- NVIC: USART1 global interrupt ---- */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* ---- Enable USART1 ---- */
    USART_Cmd(USART1, ENABLE);
}


/* ================================================================
 *  USART1 RX interrupt handler — fills line buffer
 *  Called from USART1_IRQHandler in stm32f10x_it.c
 * ================================================================ */
void GPS_UART_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART1);

        /* ---- Line complete (LF received) ---- */
        if (ch == '\n')
        {
            if (gps_line_idx > 0 && gps_line_buf[gps_line_idx - 1] == '\r')
                gps_line_idx--;                     /* strip trailing CR   */
            gps_line_buf[gps_line_idx] = '\0';      /* NUL-terminate       */
            gps_line_idx  = 0;
            gps_line_ready = 1;                     /* signal parser       */
        }
        /* ---- Discard CR, skip buffer overflow ---- */
        else if (ch == '\r')
        {
            /* nothing — LF will handle termination */
        }
        else if (gps_line_idx < GPS_LINE_BUF_SIZE - 1)
        {
            gps_line_buf[gps_line_idx++] = ch;
        }
        else
        {
            /* Buffer overflow — reset and wait for next '$' */
            gps_line_idx = 0;
        }
    }
}


/* ================================================================
 *  NMEA checksum verification
 *
 *  Format:  $...payload...*XX<CR><LF>
 *  Checksum = XOR of all bytes between '$' (exclusive) and '*'
 *             (exclusive), expressed as two hex digits.
 *
 *  Returns 1 if checksum matches, 0 otherwise.
 * ================================================================ */
static uint8_t gps_verify_checksum(const char *sentence)
{
    const char *p;
    uint8_t calc = 0;
    uint8_t expected;

    if (sentence == NULL || sentence[0] != '$')
        return 0;

    /* XOR from after '$' to before '*' (or end) */
    for (p = sentence + 1; *p != '\0' && *p != '*'; p++)
        calc ^= (uint8_t)(*p);

    if (*p != '*')
        return 0;   /* no checksum field — accept anyway for lenient parsing */

    p++;  /* skip '*' */

    /* Parse two hex digits */
    if (p[0] == '\0' || p[1] == '\0')
        return 0;

    expected = 0;
    /* High nibble */
    if (p[0] >= '0' && p[0] <= '9')       expected  = (uint8_t)((p[0] - '0') << 4);
    else if (p[0] >= 'A' && p[0] <= 'F')  expected  = (uint8_t)((p[0] - 'A' + 10) << 4);
    else if (p[0] >= 'a' && p[0] <= 'f')  expected  = (uint8_t)((p[0] - 'a' + 10) << 4);
    else return 0;

    /* Low nibble */
    if (p[1] >= '0' && p[1] <= '9')       expected |= (uint8_t)(p[1] - '0');
    else if (p[1] >= 'A' && p[1] <= 'F')  expected |= (uint8_t)(p[1] - 'A' + 10);
    else if (p[1] >= 'a' && p[1] <= 'f')  expected |= (uint8_t)(p[1] - 'a' + 10);
    else return 0;

    return (calc == expected) ? 1 : 0;
}


/* ================================================================
 *  Field extraction helper
 *
 *  Given a NMEA sentence (comma-separated), return a pointer to
 *  field N (0-based).  Returns NULL if the field doesn't exist.
 *
 *  Example:  "$GPRMC,123519,A,2232.6523,N,..."
 *            field(0) → "123519"
 *            field(1) → "A"
 * ================================================================ */
static const char *gps_get_field(const char *sentence, uint8_t n)
{
    const char *p = sentence;
    uint8_t field = 0;

    /* Skip talker + sentence ID ("$GPRMC") */
    while (*p && *p != ',') p++;
    if (*p == ',') p++;

    /* Walk through fields */
    while (field < n && *p)
    {
        while (*p && *p != ',') p++;   /* skip current field */
        if (*p == ',') { p++; field++; }
    }

    if (field != n || *p == '\0' || *p == ',' || *p == '*')
        return NULL;

    return p;
}


/* ================================================================
 *  Parse $GPRMC — Recommended Minimum Specific GNSS Data
 *
 *  Fields:
 *    0 = UTC time   (hhmmss.ss)
 *    1 = Status     (A=valid, V=warning)
 *    2 = Latitude   (ddmm.mmmm)
 *    3 = N/S indicator
 *    4 = Longitude  (dddmm.mmmm)
 *    5 = E/W indicator
 *    6 = Speed over ground (knots)
 *    7 = Course over ground (degrees)
 *    8 = Date       (ddmmyy)
 *    ...
 *    Checksum
 * ================================================================ */
static void gps_parse_gprmc(const char *sentence)
{
    const char *field;
    double nmea_val;
    char quadrant;

    /* Field 1: Status */
    field = gps_get_field(sentence, 1);
    if (field == NULL || *field != 'A')
    {
        s_parsed.valid = 0;
        return;  /* No valid fix — keep previous position data */
    }
    s_parsed.valid = 1;

    /* Field 0: UTC time (hhmmss.ss) */
    field = gps_get_field(sentence, 0);
    if (field && strlen(field) >= 6)
    {
        s_parsed.hour   = (uint8_t)((field[0]-'0')*10 + (field[1]-'0'));
        s_parsed.minute = (uint8_t)((field[2]-'0')*10 + (field[3]-'0'));
        s_parsed.second = (uint8_t)((field[4]-'0')*10 + (field[5]-'0'));
    }

    /* Field 2 & 3: Latitude */
    field = gps_get_field(sentence, 2);
    quadrant = '\0';
    {
        const char *q = gps_get_field(sentence, 3);
        if (q) quadrant = *q;
    }
    if (field && quadrant)
    {
        nmea_val = atof(field);
        s_parsed.latitude = GPS_NMEA2Decimal(nmea_val, quadrant);
    }

    /* Field 4 & 5: Longitude */
    field = gps_get_field(sentence, 4);
    quadrant = '\0';
    {
        const char *q = gps_get_field(sentence, 5);
        if (q) quadrant = *q;
    }
    if (field && quadrant)
    {
        nmea_val = atof(field);
        s_parsed.longitude = GPS_NMEA2Decimal(nmea_val, quadrant);
    }

    /* Field 6: Speed over ground (knots) */
    field = gps_get_field(sentence, 6);
    if (field) s_parsed.speed = (float)atof(field);

    /* Field 7: Course over ground (degrees) */
    field = gps_get_field(sentence, 7);
    if (field) s_parsed.course = (float)atof(field);

    /* Field 8: Date (ddmmyy) */
    field = gps_get_field(sentence, 8);
    if (field && strlen(field) >= 6)
    {
        s_parsed.day   = (uint8_t)((field[0]-'0')*10 + (field[1]-'0'));
        s_parsed.month = (uint8_t)((field[2]-'0')*10 + (field[3]-'0'));
        s_parsed.year  = (uint8_t)((field[4]-'0')*10 + (field[5]-'0'));
    }

    s_gprmc_seen = 1;
}


/* ================================================================
 *  Parse $GPGGA — Global Positioning System Fix Data
 *
 *  Fields:
 *    0 = UTC time   (hhmmss.ss)
 *    1 = Latitude   (ddmm.mmmm)
 *    2 = N/S indicator
 *    3 = Longitude  (dddmm.mmmm)
 *    4 = E/W indicator
 *    5 = Fix quality (0=invalid, 1=GPS, 2=DGPS, 6=DR)
 *    6 = Number of satellites
 *    7 = HDOP
 *    8 = Altitude   (meters, above MSL)
 *    9 = Altitude units (M)
 *    ...
 *    Checksum
 * ================================================================ */
static void gps_parse_gpgga(const char *sentence)
{
    const char *field;

    /* Field 5: Fix quality */
    field = gps_get_field(sentence, 5);
    if (field) s_parsed.fix_quality = (uint8_t)atoi(field);

    /* Field 6: Number of satellites */
    field = gps_get_field(sentence, 6);
    if (field) s_parsed.satellites = (uint8_t)atoi(field);

    /* Field 8: Altitude (meters) */
    field = gps_get_field(sentence, 8);
    if (field) s_parsed.altitude = (float)atof(field);

    s_gpgga_seen = 1;
}


/* ================================================================
 *  GPS_ProcessLine — parse one complete NMEA sentence
 *
 *  Called from the GPS thread (NOT from ISR context).
 *  Accumulates partial results; when both GPRMC and GPGGA are
 *  seen, copies to global g_gps_data and toggles g_gps_updated.
 * ================================================================ */
void GPS_ProcessLine(const char *line)
{
    if (line == NULL || line[0] != '$')
        return;

    /* ---- Verify checksum ---- */
    if (!gps_verify_checksum(line))
        return;

    /* ---- Identify sentence type ---- */
    /* Talker ID + sentence ID are at offset 1..5 (e.g. "GPRMC") */
    if (strncmp(line + 1, "GPRMC", 5) == 0 ||
        strncmp(line + 1, "GNRMC", 5) == 0)
    {
        gps_parse_gprmc(line);
    }
    else if (strncmp(line + 1, "GPGGA", 5) == 0 ||
             strncmp(line + 1, "GNGGA", 5) == 0)
    {
        gps_parse_gpgga(line);
    }

    /* ---- Both sentences received → commit to global ---- */
    if (s_gprmc_seen && s_gpgga_seen)
    {
        /* Copy accumulated data to global (atomic enough for 32-bit struct) */
        g_gps_data = s_parsed;
        g_gps_updated = 1;

        /* Reset accumulators for next cycle */
        s_gprmc_seen = 0;
        s_gpgga_seen = 0;
    }
}


/* ================================================================
 *  GPS_NMEA2Decimal — convert NMEA ddmm.mmmm format to decimal
 *
 *  NMEA lat:  ddmm.mmmm  → decimal = dd + mm.mmmm / 60
 *  NMEA lon: dddmm.mmmm  → decimal = ddd + mm.mmmm / 60
 *
 *  quadrant: 'N'/'E' → positive, 'S'/'W' → negative
 * ================================================================ */
double GPS_NMEA2Decimal(double nmea, char quadrant)
{
    double degrees;
    double minutes;
    double result;

    /* NMEA format: ddmm.mmmm (lat) or dddmm.mmmm (lon)
     *
     * Example: 2232.6523 → 22° + 32.6523' / 60
     *   degrees  = (int)(2232.6523 / 100) = 22
     *   minutes  = 2232.6523 - 2200 = 32.6523
     *   decimal  = 22 + 32.6523 / 60 = 22.544205
     *
     * Cast to int is safe here — NMEA values are always positive
     * (N/S/E/W are separate quadrant indicators).
     */
    degrees = (double)((int)(nmea / 100.0));
    minutes = nmea - degrees * 100.0;
    result  = degrees + minutes / 60.0;

    if (quadrant == 'S' || quadrant == 'W')
        result = -result;

    return result;
}


/* ================================================================
 *  GPS_CheckLine — called by gps_thread to poll for new lines
 *
 *  Returns 1 if a line was ready and processed, 0 otherwise.
 * ================================================================ */
uint8_t GPS_CheckLine(void)
{
    char local_buf[GPS_LINE_BUF_SIZE];

    if (!gps_line_ready)
        return 0;

    /* ---- Critical section: disable RXNE IRQ, copy buffer, re-enable ---- */
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    strcpy(local_buf, (const char *)gps_line_buf);
    gps_line_ready = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    GPS_ProcessLine(local_buf);
    return 1;
}

/* ================================================================
 *  GPS_Set5Hz — configure NEO-6M to 5Hz update rate via UBX
 *
 *  Default NEO-6M update rate is 1Hz (1000ms).  This sends a
 *  UBX-CFG-RATE message to change the measurement period to
 *  200ms (5Hz), giving 5 position fixes per second instead of 1.
 *
 *  UBX protocol frame:
 *    Sync: 0xB5 0x62
 *    Class: 0x06 (CFG), ID: 0x08 (RATE)
 *    Length: 6 bytes (little-endian)
 *    Payload:
 *      measRate:  200 (ms)  — U2, little-endian
 *      navRate:   1          — U2, little-endian
 *      timeRef:   0 (UTC)    — U2, little-endian
 *    Checksum: CK_A, CK_B  — 8-bit Fletcher over Class..Payload
 *
 *  Must be called AFTER GPS_UART_Init() so USART1 is ready.
 *  The NEO-6M needs about 200ms after power-up before accepting
 *  UBX commands.
 * ================================================================ */
void GPS_Set5Hz(void)
{
    uint8_t msg[] = {
        0xB5, 0x62,             /* Sync chars          */
        0x06,                   /* Class: CFG          */
        0x08,                   /* ID:   RATE          */
        0x06, 0x00,             /* Length: 6 (U2 LE)   */
        0xC8, 0x00,             /* measRate: 200ms     */
        0x01, 0x00,             /* navRate: 1          */
        0x00, 0x00,             /* timeRef: UTC        */
        0x00, 0x00              /* Checksum placeholder */
    };
    uint8_t CK_A = 0, CK_B = 0;
    uint8_t i;

    /* Compute UBX checksum over class, id, length, payload (bytes 2..11) */
    for (i = 2; i < 12; i++)
    {
        CK_A = CK_A + msg[i];
        CK_B = CK_B + CK_A;
    }
    msg[12] = CK_A;
    msg[13] = CK_B;

    /* Send the 14-byte UBX frame via USART1 (blocking) */
    for (i = 0; i < 14; i++)
    {
        USART_SendData(USART1, (uint16_t)msg[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}
