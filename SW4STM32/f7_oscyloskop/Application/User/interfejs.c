/*
 * interfejs.c
 *
 *  Created on: 18 sie 2016
 *      Author: ja
 */

#include "interfejs.h"

void wyswietl_interfejs(uint16_t x, uint16_t y, uint32_t rgb, uint8_t kanal)
{
	if(y < 8) y = 8;
	x = 0;

	//lcd_drawHline(INTERFEJS_KRAWEDZ, IKONY_KRAWEDZ, 0, 0xffff);
	//lcd_drawHline(INTERFEJS_KRAWEDZ, IKONY_KRAWEDZ, LCD_HIGH - 1, 0xffff);
	lcd_drawVline(0, LCD_HIGH - 1, IKONY_KRAWEDZ, 0xffff);
	lcd_drawVline(0, LCD_HIGH - 1, INTERFEJS_KRAWEDZ, 0xffff);
	//C
	lcd_setpixel(x, y+1, rgb);
	lcd_setpixel(x, y+2, rgb);
	lcd_setpixel(x, y+3, rgb);
	lcd_setpixel(x, y+4, rgb);
	lcd_setpixel(x, y+5, rgb);

	lcd_setpixel(x+1, y, rgb);
	lcd_setpixel(x+1, y+6, rgb);

	lcd_setpixel(x+2, y, rgb);
	lcd_setpixel(x+2, y+6, rgb);

	lcd_setpixel(x+3, y, rgb);
	lcd_setpixel(x+3, y+6, rgb);

	lcd_setpixel(x+4, y+1, rgb);
	lcd_setpixel(x+4, y+5, rgb);

	// H
	lcd_setpixel(x+6, y, rgb);
	lcd_setpixel(x+6, y+1, rgb);
	lcd_setpixel(x+6, y+2, rgb);
	lcd_setpixel(x+6, y+3, rgb);
	lcd_setpixel(x+6, y+4, rgb);
	lcd_setpixel(x+6, y+5, rgb);
	lcd_setpixel(x+6, y+6, rgb);

	lcd_setpixel(x+7, y+3, rgb);

	lcd_setpixel(x+8, y+3, rgb);

	lcd_setpixel(x+9, y+3, rgb);

	lcd_setpixel(x+10, y, rgb);
	lcd_setpixel(x+10, y+1, rgb);
	lcd_setpixel(x+10, y+2, rgb);
	lcd_setpixel(x+10, y+3, rgb);
	lcd_setpixel(x+10, y+4, rgb);
	lcd_setpixel(x+10, y+5, rgb);
	lcd_setpixel(x+10, y+6, rgb);

	//0
	lcd_setpixel(x+18, y+9, rgb);
	lcd_setpixel(x+18, y+10, rgb);
	lcd_setpixel(x+18, y+11, rgb);
	lcd_setpixel(x+18, y+12, rgb);
	lcd_setpixel(x+18, y+13, rgb);

	lcd_setpixel(x+19, y+8, rgb);
	lcd_setpixel(x+19, y+14, rgb);

	lcd_setpixel(x+20, y+8, rgb);
	lcd_setpixel(x+20, y+14, rgb);

	lcd_setpixel(x+21, y+8, rgb);
	lcd_setpixel(x+21, y+14, rgb);

	lcd_setpixel(x+22, y+9, rgb);
	lcd_setpixel(x+22, y+10, rgb);
	lcd_setpixel(x+22, y+11, rgb);
	lcd_setpixel(x+22, y+12, rgb);
	lcd_setpixel(x+22, y+13, rgb);

	//1
	lcd_setpixel(x+18, y+2-8, rgb);
	lcd_setpixel(x+18, y+6-8, rgb);

	lcd_setpixel(x+19, y+1-8, rgb);
	lcd_setpixel(x+19, y+6-8, rgb);

	lcd_setpixel(x+20, y-8, rgb);
	lcd_setpixel(x+20, y+1-8, rgb);
	lcd_setpixel(x+20, y+2-8, rgb);
	lcd_setpixel(x+20, y+3-8, rgb);
	lcd_setpixel(x+20, y+4-8, rgb);
	lcd_setpixel(x+20, y+5-8, rgb);
	lcd_setpixel(x+20, y+6-8, rgb);

	lcd_setpixel(x+21, y+6-8, rgb);

	lcd_setpixel(x+22, y+6-8, rgb);

	if(kanal == 1)
	{
		lcd_setpixel(x+12, y+2, rgb);
		lcd_setpixel(x+12, y+6, rgb);

		lcd_setpixel(x+13, y+1, rgb);
		lcd_setpixel(x+13, y+6, rgb);

		lcd_setpixel(x+14, y, rgb);
		lcd_setpixel(x+14, y+1, rgb);
		lcd_setpixel(x+14, y+2, rgb);
		lcd_setpixel(x+14, y+3, rgb);
		lcd_setpixel(x+14, y+4, rgb);
		lcd_setpixel(x+14, y+5, rgb);
		lcd_setpixel(x+14, y+6, rgb);

		lcd_setpixel(x+15, y+6, rgb);

		lcd_setpixel(x+16, y+6, rgb);


	}
	else if(kanal == 2)
	{

	}
}

void wyswietl_ikony()
{
	lcd_drawHline(IKONY_KRAWEDZ, LCD_WIDTH - 1, LCD_WIDTH - IKONY_KRAWEDZ,0xffff);
}
