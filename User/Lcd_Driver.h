#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include "stm32f10x.h"

// Color definitions (RGB565 format)
#define RED     0xf800
#define GREEN   0x07e0
#define BLUE    0x001f
#define WHITE   0xffff
#define BLACK   0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D
#define GRAY1   0x8410
#define GRAY2   0x4208

// TFT control port definitions
#define LCD_CTRLA           GPIOA       // TFT data port A
#define LCD_CTRLB           GPIOB       // TFT data port B

// Pin definitions
// SCL - PA5 (SPI Clock)
#define LCD_SCL             GPIO_Pin_5
// SDA - PA7 (SPI Data / MOSI)
#define LCD_SDA             GPIO_Pin_7
// CS  - PA4 (Chip Select)
#define LCD_CS              GPIO_Pin_4
// BL  - PA12 (Backlight) — moved from PB10 to free USART3
#define LCD_LED             GPIO_Pin_12
// RS  - PB1 (Data/Command, also called DC)
#define LCD_RS              GPIO_Pin_1
// RST - PB0 (Reset)
#define LCD_RST             GPIO_Pin_0

// Control macros - Set (HIGH) using BSRR register
#define LCD_SCL_SET         LCD_CTRLA->BSRR = LCD_SCL
#define LCD_SDA_SET         LCD_CTRLA->BSRR = LCD_SDA
#define LCD_CS_SET          LCD_CTRLA->BSRR = LCD_CS

#define LCD_LED_SET         LCD_CTRLA->BSRR = LCD_LED
#define LCD_RS_SET          LCD_CTRLB->BSRR = LCD_RS
#define LCD_RST_SET         LCD_CTRLB->BSRR = LCD_RST

// Control macros - Clear (LOW) using BRR register
#define LCD_SCL_CLR         LCD_CTRLA->BRR = LCD_SCL
#define LCD_SDA_CLR         LCD_CTRLA->BRR = LCD_SDA
#define LCD_CS_CLR          LCD_CTRLA->BRR = LCD_CS

#define LCD_LED_CLR         LCD_CTRLA->BRR = LCD_LED
#define LCD_RST_CLR         LCD_CTRLB->BRR = LCD_RST
#define LCD_RS_CLR          LCD_CTRLB->BRR = LCD_RS

// Function prototypes
void LCD_GPIO_Init(void);
void Lcd_WriteIndex(u8 Index);
void Lcd_WriteData(u8 Data);
void Lcd_WriteReg(u8 Index, u8 Data);
u16 Lcd_ReadReg(u8 LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(u16 Color);
void Lcd_SetXY(u16 x, u16 y);
void Gui_DrawPoint(u16 x, u16 y, u16 Data);
unsigned int Lcd_ReadPoint(u16 x, u16 y);
void Lcd_SetRegion(u16 x_start, u16 y_start, u16 x_end, u16 y_end);
void LCD_WriteData_16Bit(u16 Data);

#endif /* __LCD_DRIVER_H */
