/*
 * LCD.c
 *
 *  Created on: Aug 5, 2025
 *      Author: krushna
 */

#include "I2C.h"
#include "LCD.h"


void LcdInit(uint8_t addr)
{

		// lcd initialization
		DelayMs(20);
		LcdWriteNibble(addr,LCD_CMD, 0x03);
		DelayMs(5);
		LcdWriteNibble(addr,LCD_CMD, 0x03);
		DelayMs(1);
		LcdWriteNibble(addr,LCD_CMD, 0x03);
		DelayMs(1);
		LcdWriteNibble(addr,LCD_CMD, 0x02);
		DelayMs(1);

		LcdWriteByte(addr,LCD_CMD, LCD_FNSET_2LINE);
		LcdWriteByte(addr,LCD_CMD, LCD_DISP_OFF);
		LcdWriteByte(addr,LCD_CMD, LCD_CLEAR);
		LcdWriteByte(addr,LCD_CMD, LCD_ENTRYMODE);
		LcdWriteByte(addr,LCD_CMD, LCD_DISP_ON);



}

void LcdWriteNibble(uint8_t addr,uint8_t rs, uint8_t data) {
	uint8_t rsFlag = rs == LCD_DATA ? BV(LCD_RS_Pos) : 0;
	uint8_t val = (data << 4) | rsFlag | BV(LCD_BL_Pos) | BV(LCD_EN_Pos) ;
	I2CWrite(addr, val);
	DelayMs(1);
	val = (data << 4) | rsFlag | BV(LCD_BL_Pos);
	I2CWrite(addr, val);
}

void LcdWriteByte(uint8_t addr,uint8_t rs, uint8_t data) {
	// divide data into two nibbles
	uint8_t high = data >> 4, low = data & 0x0F;
	LcdWriteNibble(addr,rs, high);
	LcdWriteNibble(addr,rs, low);
	// wait for busy flag -- optional for low speed uc
	//DelayMs(1);

	if (data == LCD_CLEAR || data == 0x02)
	        DelayMs(2);     // 1.52ms minimum
	    else
	        DelayMs(1);
}

void LcdPuts(uint8_t addr,uint8_t line, char *str) {
	// set line start addr
	LcdWriteByte(addr,LCD_CMD, line);
	// write chars one by one
	for(int i=0; str[i] != '\0'; i++)
		LcdWriteByte(addr,LCD_DATA, str[i]);
	DelayMs(2);
}












