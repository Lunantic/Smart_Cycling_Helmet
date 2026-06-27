/**
 ******************************************************************************
 * @file    GUI.c
 * @brief   GUI library for TFT LCD (line, circle, box, font drawing)
 *          Adapted for RT-Thread on STM32F103C8T6
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "font.h"

// Convert BGR format to RGB format (used when display mode is BGR)
u16 LCD_BGR2RGB(u16 c)
{
    u16  r, g, b, rgb;
    b = (c >> 0) & 0x1f;
    g = (c >> 5) & 0x3f;
    r = (c >> 11) & 0x1f;
    rgb = (b << 11) + (g << 5) + (r << 0);
    return(rgb);
}


// Bresenham circle drawing algorithm
void Gui_Circle(u16 X, u16 Y, u16 R, u16 fc)
{
    unsigned short  a, b;
    int c;
    a = 0;
    b = R;
    c = 3 - 2 * R;
    while (a < b)
    {
        Gui_DrawPoint(X + a, Y + b, fc);
        Gui_DrawPoint(X - a, Y + b, fc);
        Gui_DrawPoint(X + a, Y - b, fc);
        Gui_DrawPoint(X - a, Y - b, fc);
        Gui_DrawPoint(X + b, Y + a, fc);
        Gui_DrawPoint(X - b, Y + a, fc);
        Gui_DrawPoint(X + b, Y - a, fc);
        Gui_DrawPoint(X - b, Y - a, fc);

        if(c < 0) c = c + 4 * a + 6;
        else
        {
            c = c + 4 * (a - b) + 10;
            b -= 1;
        }
        a += 1;
    }
    if (a == b)
    {
        Gui_DrawPoint(X + a, Y + b, fc);
        Gui_DrawPoint(X + a, Y + b, fc);
        Gui_DrawPoint(X + a, Y - b, fc);
        Gui_DrawPoint(X - a, Y - b, fc);
        Gui_DrawPoint(X + b, Y + a, fc);
        Gui_DrawPoint(X - b, Y + a, fc);
        Gui_DrawPoint(X + b, Y - a, fc);
        Gui_DrawPoint(X - b, Y - a, fc);
    }
}


// Bresenham line drawing algorithm
void Gui_DrawLine(u16 x0, u16 y0, u16 x1, u16 y1, u16 Color)
{
    int dx, dy, dx2, dy2,
        x_inc, y_inc,
        error, index;

    Lcd_SetXY(x0, y0);
    dx = x1 - x0;
    dy = y1 - y0;

    if (dx >= 0)
    {
        x_inc = 1;
    }
    else
    {
        x_inc = -1;
        dx    = -dx;
    }

    if (dy >= 0)
    {
        y_inc = 1;
    }
    else
    {
        y_inc = -1;
        dy    = -dy;
    }

    dx2 = dx << 1;
    dy2 = dy << 1;

    if (dx > dy)
    {
        error = dy2 - dx;
        for (index = 0; index <= dx; index++)
        {
            Gui_DrawPoint(x0, y0, Color);

            if (error >= 0)
            {
                error -= dx2;
                y0 += y_inc;
            }
            error += dy2;
            x0 += x_inc;
        }
    }
    else
    {
        error = dx2 - dy;
        for (index = 0; index <= dy; index++)
        {
            Gui_DrawPoint(x0, y0, Color);

            if (error >= 0)
            {
                error -= dy2;
                x0 += x_inc;
            }
            error += dx2;
            y0 += y_inc;
        }
    }
}


// Draw a 3D box
void Gui_box(u16 x, u16 y, u16 w, u16 h, u16 bc)
{
    Gui_DrawLine(x, y, x + w, y, 0xEF7D);
    Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
    Gui_DrawLine(x, y + h, x + w, y + h, 0x2965);
    Gui_DrawLine(x, y, x, y + h, 0xEF7D);
    Gui_DrawLine(x + 1, y + 1, x + 1 + w - 2, y + 1 + h - 2, bc);
}


// Draw a 2D box with mode (0=raised, 1=sunken, 2=flat)
void Gui_box2(u16 x, u16 y, u16 w, u16 h, u8 mode)
{
    if (mode == 0) {
        Gui_DrawLine(x, y, x + w, y, 0xEF7D);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
        Gui_DrawLine(x, y + h, x + w, y + h, 0x2965);
        Gui_DrawLine(x, y, x, y + h, 0xEF7D);
    }
    if (mode == 1) {
        Gui_DrawLine(x, y, x + w, y, 0x2965);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xEF7D);
        Gui_DrawLine(x, y + h, x + w, y + h, 0xEF7D);
        Gui_DrawLine(x, y, x, y + h, 0x2965);
    }
    if (mode == 2) {
        Gui_DrawLine(x, y, x + w, y, 0xffff);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xffff);
        Gui_DrawLine(x, y + h, x + w, y + h, 0xffff);
        Gui_DrawLine(x, y, x, y + h, 0xffff);
    }
}


// Display a "pressed" (sunken) button
void DisplayButtonDown(u16 x1, u16 y1, u16 x2, u16 y2)
{
    Gui_DrawLine(x1,     y1,     x2, y1,     GRAY2);
    Gui_DrawLine(x1 + 1, y1 + 1, x2, y1 + 1, GRAY1);
    Gui_DrawLine(x1,     y1,     x1, y2,     GRAY2);
    Gui_DrawLine(x1 + 1, y1 + 1, x1 + 1, y2, GRAY1);
    Gui_DrawLine(x1,     y2,     x2, y2,     WHITE);
    Gui_DrawLine(x2,     y1,     x2, y2,     WHITE);
}


// Display a "released" (raised) button
void DisplayButtonUp(u16 x1, u16 y1, u16 x2, u16 y2)
{
    Gui_DrawLine(x1,     y1,     x2, y1,     WHITE);
    Gui_DrawLine(x1,     y1,     x1, y2,     WHITE);

    Gui_DrawLine(x1 + 1, y2 - 1, x2, y2 - 1, GRAY1);
    Gui_DrawLine(x1,     y2,     x2, y2,     GRAY2);
    Gui_DrawLine(x2 - 1, y1 + 1, x2 - 1, y2, GRAY1);
    Gui_DrawLine(x2,     y1,     x2, y2,     GRAY2);
}


// Draw GBK font (16x16) with mixed ASCII and Chinese characters
void Gui_DrawFont_GBK16(u16 x, u16 y, u16 fc, u16 bc, u8 *s)
{
    unsigned char i, j;
    unsigned short k, x0;
    x0 = x;

    while(*s)
    {
        if((*s) < 128)
        {
            k = *s;
            if (k == 13)
            {
                x = x0;
                y += 16;
            }
            else
            {
                if (k > 32) k -= 32; else k = 0;

                for(i = 0; i < 16; i++)
                    for(j = 0; j < 8; j++)
                    {
                        if(asc16[k * 16 + i] & (0x80 >> j))
                            Gui_DrawPoint(x + j, y + i, fc);
                        else
                        {
                            if (fc != bc) Gui_DrawPoint(x + j, y + i, bc);
                        }
                    }
                x += 8;
            }
            s++;
        }
        else
        {
            for (k = 0; k < hz16_num; k++)
            {
                if ((hz16[k].Index[0] == *(s)) && (hz16[k].Index[1] == *(s + 1)))
                {
                    for(i = 0; i < 16; i++)
                    {
                        for(j = 0; j < 8; j++)
                        {
                            if(hz16[k].Msk[i * 2] & (0x80 >> j))
                                Gui_DrawPoint(x + j, y + i, fc);
                            else {
                                if (fc != bc) Gui_DrawPoint(x + j, y + i, bc);
                            }
                        }
                        for(j = 0; j < 8; j++)
                        {
                            if(hz16[k].Msk[i * 2 + 1] & (0x80 >> j))
                                Gui_DrawPoint(x + j + 8, y + i, fc);
                            else
                            {
                                if (fc != bc) Gui_DrawPoint(x + j + 8, y + i, bc);
                            }
                        }
                    }
                }
            }
            s += 2; x += 16;
        }
    }
}


// Draw GBK font (24x24)
void Gui_DrawFont_GBK24(u16 x, u16 y, u16 fc, u16 bc, u8 *s)
{
    unsigned char i, j;
    unsigned short k;

    while(*s)
    {
        if( *s < 0x80 )
        {
            k = *s;
            if (k > 32) k -= 32; else k = 0;

            for(i = 0; i < 16; i++)
                for(j = 0; j < 8; j++)
                {
                    if(asc16[k * 16 + i] & (0x80 >> j))
                        Gui_DrawPoint(x + j, y + i, fc);
                    else
                    {
                        if (fc != bc) Gui_DrawPoint(x + j, y + i, bc);
                    }
                }
            s++; x += 8;
        }
        else
        {
            for (k = 0; k < hz24_num; k++)
            {
                if ((hz24[k].Index[0] == *(s)) && (hz24[k].Index[1] == *(s + 1)))
                {
                    for(i = 0; i < 24; i++)
                    {
                        for(j = 0; j < 8; j++)
                        {
                            if(hz24[k].Msk[i * 3] & (0x80 >> j))
                                Gui_DrawPoint(x + j, y + i, fc);
                            else
                            {
                                if (fc != bc) Gui_DrawPoint(x + j, y + i, bc);
                            }
                        }
                        for(j = 0; j < 8; j++)
                        {
                            if(hz24[k].Msk[i * 3 + 1] & (0x80 >> j))
                                Gui_DrawPoint(x + j + 8, y + i, fc);
                            else {
                                if (fc != bc) Gui_DrawPoint(x + j + 8, y + i, bc);
                            }
                        }
                        for(j = 0; j < 8; j++)
                        {
                            if(hz24[k].Msk[i * 3 + 2] & (0x80 >> j))
                                Gui_DrawPoint(x + j + 16, y + i, fc);
                            else
                            {
                                if (fc != bc) Gui_DrawPoint(x + j + 16, y + i, bc);
                            }
                        }
                    }
                }
            }
            s += 2; x += 24;
        }
    }
}


// Draw 32x32 numeric font
void Gui_DrawFont_Num32(u16 x, u16 y, u16 fc, u16 bc, u16 num)
{
    unsigned char i, j, k, c;

    for(i = 0; i < 32; i++)
    {
        for(j = 0; j < 4; j++)
        {
            c = *(sz32 + num * 32 * 4 + i * 4 + j);
            for (k = 0; k < 8; k++)
            {
                if(c & (0x80 >> k))
                    Gui_DrawPoint(x + j * 8 + k, y + i, fc);
                else {
                    if (fc != bc) Gui_DrawPoint(x + j * 8 + k, y + i, bc);
                }
            }
        }
    }
}
