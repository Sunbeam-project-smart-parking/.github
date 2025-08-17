/*
 * LCD.h
 *
 *  Created on: Aug 5, 2025
 *      Author: krushna
 */

#ifndef LCD_H_
#define LCD_H_

#include "I2C.h"

#define SLOT_1	0x4E
#define SLOT_2	0x4A

void LcdInit(uint8_t addr);
void LcdWriteNibble(uint8_t addr,uint8_t rs, uint8_t data);
void LcdWriteByte(uint8_t addr,uint8_t rs, uint8_t data);
void LcdPuts(uint8_t addr,uint8_t line, char *str);

#define LCD_CMD			0
#define LCD_DATA		1
#define LCD_LINE1		0x80
#define LCD_LINE2		0xC0
#define LCD_CLEAR		0x01
#define LCD_ENTRYMODE	0x06
#define LCD_DISP_OFF	0x08
#define LCD_DISP_ON		0x0C
#define LCD_FNSET_1LINE	0x20
#define LCD_FNSET_2LINE	0x28

#define LCD_RS_Pos		0
#define LCD_RW_Pos		1
#define LCD_EN_Pos		2
#define LCD_BL_Pos		3



#endif /* LCD_H_ */
