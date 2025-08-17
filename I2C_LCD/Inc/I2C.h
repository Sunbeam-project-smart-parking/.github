/*
 * I2C.h
 *
 *  Created on: Aug 5, 2025
 *      Author: krushna
 */

#ifndef I2C_H_
#define I2C_H_

#include<stm32f4xx.h>

void I2CInit(void);
void I2CStart(void);
void I2CRepeateStart(void);
void I2CStop(void);
void I2CSendAddr(uint8_t addr);
void I2CSendData(uint8_t data);
uint8_t I2CRecDataAck(void);
uint8_t I2CRecDataNAck(void);
void I2CWrite(uint8_t addr ,uint8_t data);


#endif /* I2C_H_ */
