/*
 * interfejs.h
 *
 *  Created on: 18 sie 2016
 *      Author: ja
 */

#ifndef APPLICATION_INC_INTERFEJS_H_
#define APPLICATION_INC_INTERFEJS_H_

#include "lcd.h"

#define INTERFEJS_KRAWEDZ 30
#define IKONY_KRAWEDZ 430

void wyswietl_interfejs(uint16_t x, uint16_t y, uint32_t rgb, uint8_t kanal);
void wyswietl_ikony();

#endif /* APPLICATION_INC_INTERFEJS_H_ */
