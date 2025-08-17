/*
 * I2C.c
 *
 *  Created on: Aug 5, 2025
 *      Author: krushna
 */


// NOTE  PB-8 SCL  & PB-9  SDA

#include "I2C.h"


void I2CInit(void)
{

	//  gpio config.
	//selecting i2c ->1
	// Pin -> PB-8 & PB-9

	RCC-> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB -> MODER |= BV(8*2+1) | BV(9*2+1);
	GPIOB -> MODER &= ~(BV(8*2) | BV(9*2));
	GPIOB -> PUPDR &= ~(BV(8*2+1) | BV(8*2) | BV(9*2+1) | BV(9*2));
	GPIOB -> OTYPER |= BV(8) | BV(9);
	GPIOB -> AFR[1] = (4 << (4 * 1)) |( 4 <<(4 * 0));


	RCC-> APB1ENR |= RCC_APB1ENR_I2C1EN;
	I2C1 -> CR1 |= I2C_CR1_SWRST;
	I2C1 -> CR1 = 0;
	I2C1 -> CR2 |= 16 << I2C_CR2_FREQ_Pos;
	I2C1 -> CCR = 80;
	I2C1 -> CCR &= ~I2C_CCR_FS;
	I2C1 -> TRISE = 17;
	I2C1 -> CR1 |= I2C_CR1_ACK;
	I2C1 -> CR1 |= I2C_CR1_PE;

}

void I2CStart(void)
{
	I2C1-> CR1 |= I2C_CR1_START;

	while(!(I2C1 ->SR1 & I2C_SR1_SB));
}

void I2CStop(void)
{
	I2C1-> CR1 |= I2C_CR1_STOP;
	while(I2C1 -> SR2 & I2C_SR2_BUSY);
}

void I2CRepeateStart(void)
{
	I2CStart();
}

void I2CSendAddr(uint8_t addr)
{
	I2C1->DR = addr;
	while(! (I2C1 -> SR1 & I2C_SR1_ADDR));
	(void)I2C1 ->SR1;
	(void)I2C1 ->SR2;
}

void I2CSendData(uint8_t data)
{
	while(! (I2C1 ->SR1 & I2C_SR1_TXE));
	I2C1 -> DR = data;

	while( !(I2C1 -> SR1 & I2C_SR1_BTF));
}

uint8_t I2CRecDataAck(void)
{
	I2C1 -> CR1 |= I2C_CR1_ACK | I2C_CR1_POS;

	while(! (I2C1 -> SR1 & I2C_SR1_RXNE));

	return I2C1 ->DR;
}

uint8_t I2CRecDataNAck(void)
{
	I2C1 -> CR1 &= ~(I2C_CR1_ACK | I2C_CR1_POS);

	while(! (I2C1 -> SR1  & I2C_SR1_RXNE));
	return I2C1  ->DR;
}

void I2CWrite(uint8_t addr ,uint8_t data)
{
	I2CStart();
	I2CSendAddr(addr);
	I2CSendData(data);
	I2CStop();
}





















