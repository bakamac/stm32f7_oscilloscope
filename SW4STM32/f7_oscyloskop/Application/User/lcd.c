/*
 * lcd.c
 *
 *  Created on: 15 sie 2016
 *      Author: ja
 */

#include "lcd.h"


void lcd_init()
{
    hltdc.LayerCfg[1].WindowX0 = 0;
    hltdc.LayerCfg[1].WindowX1 = 480;
    hltdc.LayerCfg[1].WindowY0 = 0;
    hltdc.LayerCfg[1].WindowY1 = 272;

    /* Pixel Format configuration*/
    hltdc.LayerCfg[1].PixelFormat = LTDC_PIXEL_FORMAT_RGB565;

    /* Start Address configuration : frame buffer is located at FLASH memory */
    hltdc.LayerCfg[1].FBStartAdress = (uint32_t)&lcd_buf;

    /* Alpha constant (255 == totally opaque) */
    hltdc.LayerCfg[1].Alpha = 255;

    /* Default Color configuration (configure A,R,G,B component values) : no background color */
    hltdc.LayerCfg[1].Alpha0 = 0; /* fully transparent */
    hltdc.LayerCfg[1].Backcolor.Blue = 0;
    hltdc.LayerCfg[1].Backcolor.Green = 0;
    hltdc.LayerCfg[1].Backcolor.Red = 0;

    /* Configure blending factors */
    hltdc.LayerCfg[1].BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    hltdc.LayerCfg[1].BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;

    /* Configure the number of lines and number of pixels per line */
    hltdc.LayerCfg[1].ImageWidth  = 480;
    hltdc.LayerCfg[1].ImageHeight = 272;

    //background color
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
    //lcd enable
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
}

void lcd_clear()
{
	uint16_t i, j;
	for(i = 0; i < LCD_WIDTH; i++)
		for(j = 0; j < LCD_HIGH; j++)
			lcd_setpixel(i, j, 0);
}

void lcd_setpixel(uint16_t x, uint16_t y, uint32_t rgb)
{
	x %= LCD_WIDTH;
	y %= LCD_HIGH;
	*(__IO uint16_t*) (hltdc.LayerCfg[1].FBStartAdress + (2*(y * LCD_WIDTH + x))) = (uint16_t)rgb;
}

void lcd_drawVline(uint16_t ystart, uint16_t ystop, uint16_t x, uint32_t rgb)
{
	uint16_t tmp, i;

	if(ystart > ystop)
	{
		tmp = ystop;
		ystop = ystart;
		ystart = tmp;
	}
	for(i = ystart; i <= ystop; i++)
		lcd_setpixel(x, i, rgb);
}

void lcd_drawHline(uint16_t xstart, uint16_t xstop, uint16_t y, uint32_t rgb)
{
	uint16_t tmp, i;

	if(xstart > xstop)
	{
		tmp = xstop;
		xstop = xstart;
		xstart = tmp;
	}
	for(i = xstart; i <= xstop; i++)
		lcd_setpixel(i, y, rgb);
}

void lcd_drawline(uint16_t xstart, uint16_t ystart, uint16_t xstop, uint16_t ystop, uint32_t rgb)
{
	  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	  yinc1 = 0, yinc2 = 0, den = 0, num = 0, num_add = 0, num_pixels = 0,
	  curpixel = 0;

	  deltax = ABS(xstop - xstart);        /* The difference between the x's */
	  deltay = ABS(ystop - ystart);        /* The difference between the y's */
	  x = xstart;                       /* Start x off at the first pixel */
	  y = ystart;                       /* Start y off at the first pixel */

	  if (xstop)                 /* The x-values are increasing */
	  {
	    xinc1 = 1;
	    xinc2 = 1;
	  }
	  else                          /* The x-values are decreasing */
	  {
	    xinc1 = -1;
	    xinc2 = -1;
	  }

	  if (ystop >= ystart)                 /* The y-values are increasing */
	  {
	    yinc1 = 1;
	    yinc2 = 1;
	  }
	  else                          /* The y-values are decreasing */
	  {
	    yinc1 = -1;
	    yinc2 = -1;
	  }

	  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	  {
	    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
	    yinc2 = 0;                  /* Don't change the y for every iteration */
	    den = deltax;
	    num = deltax / 2;
	    num_add = deltay;
	    num_pixels = deltax;         /* There are more x-values than y-values */
	  }
	  else                          /* There is at least one y-value for every x-value */
	  {
	    xinc2 = 0;                  /* Don't change the x for every iteration */
	    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
	    den = deltay;
	    num = deltay / 2;
	    num_add = deltax;
	    num_pixels = deltay;         /* There are more y-values than x-values */
	  }

	  for (curpixel = 0; curpixel <= num_pixels; curpixel++)
	  {
		lcd_setpixel(x, y, rgb);   /* Draw the current pixel */
	    num += num_add;                            /* Increase the numerator by the top of the fraction */
	    if (num >= den)                           /* Check if numerator >= denominator */
	    {
	      num -= den;                             /* Calculate the new numerator value */
	      x += xinc1;                             /* Change the x as appropriate */
	      y += yinc1;                             /* Change the y as appropriate */
	    }
	    x += xinc2;                               /* Change the x as appropriate */
	    y += yinc2;                               /* Change the y as appropriate */
	  }
}
