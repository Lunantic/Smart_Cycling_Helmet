/**
 ******************************************************************************
 * @file    Lcd_Driver.c
 * @brief   TFT LCD Driver for ST7735R (1.44 inch, 128x160)
 *          Adapted for RT-Thread on STM32F103C8T6
 *
 * Pin connections:
 *          GND   -> Power GND
 *          VCC   -> 5V or 3.3V
 *          SCL   -> PA5 (SCL/SCK)
 *          SDA   -> PA7 (SDA/MOSI)
 *          RES   -> PB0 (Reset)
 *          DC    -> PB1 (Data/Command)
 *          CS    -> PA4 (Chip Select)
 *          BL    -> PA12 (Backlight, moved from PB10 to free USART3)
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "LCD_Config.h"
#include <rtthread.h>

// LCD GPIO Initialization
void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    // Enable GPIOB clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB0 (RST), PB1 (DC/RS)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  /* PB10→PA12 for BL */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA4 (CS), PA5 (SCL), PA7 (SDA)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// Software SPI: write one 8-bit byte
void  SPI_WriteData(u8 Data)
{
    unsigned char i = 0;
    for(i = 8; i > 0; i--)
    {
        if(Data & 0x80)
            LCD_SDA_SET;
        else
            LCD_SDA_CLR;

        LCD_SCL_CLR;
        LCD_SCL_SET;
        Data <<= 1;
    }
}

// Write an 8-bit command/index to LCD
void Lcd_WriteIndex(u8 Index)
{
    LCD_CS_CLR;
    LCD_RS_CLR;         // RS=0: command mode
    SPI_WriteData(Index);
    LCD_CS_SET;
}

// Write an 8-bit data to LCD
void Lcd_WriteData(u8 Data)
{
    LCD_CS_CLR;
    LCD_RS_SET;         // RS=1: data mode
    SPI_WriteData(Data);
    LCD_CS_SET;
}

// Write a 16-bit data (high byte first)
void LCD_WriteData_16Bit(u16 Data)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_WriteData(Data >> 8);    // High byte
    SPI_WriteData(Data);          // Low byte
    LCD_CS_SET;
}

void Lcd_WriteReg(u8 Index, u8 Data)
{
    Lcd_WriteIndex(Index);
    Lcd_WriteData(Data);
}

void Lcd_Reset(void)
{
    LCD_RST_CLR;
    rt_thread_mdelay(100);
    LCD_RST_SET;
    rt_thread_mdelay(50);
}

// LCD Init For 1.44 Inch LCD Panel with ST7735R.
void Lcd_Init(void)
{
    LCD_GPIO_Init();
    Lcd_Reset(); // Reset before LCD Init.

    // LCD Init For 1.44Inch LCD Panel with ST7735R.
    Lcd_WriteIndex(0x11);   // Sleep exit
    rt_thread_mdelay(120);

    // ST7735R Frame Rate
    Lcd_WriteIndex(0xB1);
    Lcd_WriteData(0x01);
    Lcd_WriteData(0x2C);
    Lcd_WriteData(0x2D);

    Lcd_WriteIndex(0xB2);
    Lcd_WriteData(0x01);
    Lcd_WriteData(0x2C);
    Lcd_WriteData(0x2D);

    Lcd_WriteIndex(0xB3);
    Lcd_WriteData(0x01);
    Lcd_WriteData(0x2C);
    Lcd_WriteData(0x2D);
    Lcd_WriteData(0x01);
    Lcd_WriteData(0x2C);
    Lcd_WriteData(0x2D);

    Lcd_WriteIndex(0xB4);   // Column inversion
    Lcd_WriteData(0x07);

    // ST7735R Power Sequence
    Lcd_WriteIndex(0xC0);
    Lcd_WriteData(0xA2);
    Lcd_WriteData(0x02);
    Lcd_WriteData(0x84);
    Lcd_WriteIndex(0xC1);
    Lcd_WriteData(0xC5);

    Lcd_WriteIndex(0xC2);
    Lcd_WriteData(0x0A);
    Lcd_WriteData(0x00);

    Lcd_WriteIndex(0xC3);
    Lcd_WriteData(0x8A);
    Lcd_WriteData(0x2A);
    Lcd_WriteIndex(0xC4);
    Lcd_WriteData(0x8A);
    Lcd_WriteData(0xEE);

    Lcd_WriteIndex(0xC5);   // VCOM
    Lcd_WriteData(0x0E);

    Lcd_WriteIndex(0x36);   // MX, MY, RGB mode
    Lcd_WriteData(0xC0);

    // ST7735R Gamma Sequence
    Lcd_WriteIndex(0xe0);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x1a);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x18);
    Lcd_WriteData(0x2f);
    Lcd_WriteData(0x28);
    Lcd_WriteData(0x20);
    Lcd_WriteData(0x22);
    Lcd_WriteData(0x1f);
    Lcd_WriteData(0x1b);
    Lcd_WriteData(0x23);
    Lcd_WriteData(0x37);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x07);
    Lcd_WriteData(0x02);
    Lcd_WriteData(0x10);

    Lcd_WriteIndex(0xe1);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x1b);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x17);
    Lcd_WriteData(0x33);
    Lcd_WriteData(0x2c);
    Lcd_WriteData(0x29);
    Lcd_WriteData(0x2e);
    Lcd_WriteData(0x30);
    Lcd_WriteData(0x30);
    Lcd_WriteData(0x39);
    Lcd_WriteData(0x3f);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x07);
    Lcd_WriteData(0x03);
    Lcd_WriteData(0x10);

    // Column address set
    Lcd_WriteIndex(0x2a);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x7f);

    // Row address set
    Lcd_WriteIndex(0x2b);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x9f);

    Lcd_WriteIndex(0xF0);   // Enable test command
    Lcd_WriteData(0x01);
    Lcd_WriteIndex(0xF6);   // Disable ram power save mode
    Lcd_WriteData(0x00);

    Lcd_WriteIndex(0x3A);   // 65k mode (16-bit/pixel)
    Lcd_WriteData(0x05);

    Lcd_WriteIndex(0x29);   // Display on
}


/*************************************************
Function: Lcd_SetRegion
Description: Set the display region in GRAM for writing
Input: x_start, y_start, x_end, y_end coordinates
Return: none
*************************************************/
void Lcd_SetRegion(u16 x_start, u16 y_start, u16 x_end, u16 y_end)
{
    Lcd_WriteIndex(0x2a);
    Lcd_WriteData(0x00);
    Lcd_WriteData(x_start);
    Lcd_WriteData(0x00);
    Lcd_WriteData(x_end + 2);

    Lcd_WriteIndex(0x2b);
    Lcd_WriteData(0x00);
    Lcd_WriteData(y_start);
    Lcd_WriteData(0x00);
    Lcd_WriteData(y_end + 1);

    Lcd_WriteIndex(0x2c);
}


/*************************************************
Function: Lcd_SetXY
Description: Set LCD display coordinate
Input: x, y coordinates
Return: none
*************************************************/
void Lcd_SetXY(u16 x, u16 y)
{
    Lcd_SetRegion(x, y, x, y);
}


/*************************************************
Function: Gui_DrawPoint
Description: Draw a single pixel
Input: x, y coordinates, Data (16-bit color)
Return: none
*************************************************/
void Gui_DrawPoint(u16 x, u16 y, u16 Data)
{
    Lcd_SetRegion(x, y, x + 1, y + 1);
    LCD_WriteData_16Bit(Data);
}


/*************************************************
Function: Lcd_Clear
Description: Clear entire screen with a color
Input: 16-bit Color
Return: none
*************************************************/
void Lcd_Clear(u16 Color)
{
    unsigned int i, m;
    Lcd_SetRegion(0, 0, X_MAX_PIXEL - 1, Y_MAX_PIXEL - 1);
    Lcd_WriteIndex(0x2C);
    for(i = 0; i < X_MAX_PIXEL; i++)
        for(m = 0; m < Y_MAX_PIXEL; m++)
        {
            LCD_WriteData_16Bit(Color);
        }
}
