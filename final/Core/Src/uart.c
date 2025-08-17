/*
 * uart.c
 *
 *  Created on: Aug 8, 2025
 *      Author: krushna
 */


/*
 * uart.c
 *
 *  Created on: Mar 29, 2025
 *      Author: admin
 */

#include "uart.h"

void UartInit(uint32_t baud) {
	// gpio settings
	// enable gpio clock - AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// set gpio mode as alt fn - MODER = 10
	GPIOB->MODER |= BV(2*10+1) | BV(2*11+1);
	GPIOB->MODER &= ~(BV(2*10) | BV(2*11));

	// set alt fn - AFRH --> AF7 = USART3
	GPIOB->AFR[1] &= ~((0xF << (4*(10-8))) | (0xF << (4*(11-8)))); // clear
	GPIOB->AFR[1] |= (7 << (4*(10-8))) | (7 << (4*(11-8)));

	// disable pull up & pull down - PUPDR
	GPIOB->PUPDR &= ~(BV(2*10) | BV(2*11) | BV(2*10+1) | BV(2*11+1));

	// uart settings
	// enable clock - APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// CR1 = 8N1, Tx En
	USART3->CR1 = USART_CR1_TE;
	// CR2 = 1 stop bit (00) -- default
	USART3->CR2 &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);

	// set baud rate - BRR (assuming 16 MHz APB1 clock)
	if(baud == 9600)
		USART3->BRR = 0x0683;
	else if(baud == 38400)
		USART3->BRR = 0x01A1;
	else if(baud == 115200)
		USART3->BRR = 0x008B;

	// enable uart - CR1
	USART3->CR1 |= USART_CR1_UE;

	// enable uart intr in NVIC - ISER regr
	NVIC_EnableIRQ(USART3_IRQn);
}

static int tx_index;
static char *tx_str;
volatile static int tx_completed = 1;

void UartPuts(char *str) {
	// wait until prev string is transmitted
	while(tx_completed == 0)
		;
	// write first char into TDR
	tx_str = str;
	tx_index = 0;
	tx_completed = 0; // start new str tx
	USART3->DR = tx_str[tx_index];
	// enable tx intr - CR1
	USART3->CR1 |= USART_CR1_TXEIE;
}

void USART3_IRQHandler(void) {
	// confirm if tx intr is raised -- look at TXE flag in SR
	if((USART3->SR & USART_SR_TXE) != 0) {
		// go to next char
		tx_index++;
		// if \0 char, disable tx intr - CR1
		if(tx_str[tx_index] == '\0') {
			USART3->CR1 &= ~USART_CR1_TXEIE;
			tx_completed = 1;
		// else send next char to TDR
		} else
			USART3->DR = tx_str[tx_index];
	}
	// check if rx intr is raised (if rx intr enabled and char received)
	// if((USART3->SR & USART_SR_RXNE) != 0) {
	//     // write rx intr handling logic here
	// }
}
