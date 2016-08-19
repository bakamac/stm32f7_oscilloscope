/*
 * lcd.h
 *
 *  Created on: 15 sie 2016
 *      Author: ja
 */

#ifndef APPLICATION_INC_LCD_H_
#define APPLICATION_INC_LCD_H_

#include "stm32f7xx_hal.h"
#include "ltdc.h"
#include "gpio.h"

#define ABS(X)  ((X) > 0 ? (X) : -(X))


#define LCD_ADDR  0b01110000


#define LCD_WIDTH 480
#define LCD_HIGH  272

uint32_t lcd_buf[65280];

void lcd_init();

void lcd_clear();

void lcd_setpixel(uint16_t x, uint16_t y, uint32_t rgb);
void lcd_drawline(uint16_t xstart, uint16_t ystart, uint16_t xstop, uint16_t ystop, uint32_t rgb);
void lcd_drawVline(uint16_t xstart, uint16_t xstop, uint16_t y, uint32_t rgb);
void lcd_drawHline(uint16_t xstart, uint16_t xstop, uint16_t y, uint32_t rgb);

#endif /* APPLICATION_INC_LCD_H_ */
