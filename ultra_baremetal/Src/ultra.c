/*
 * ultra.c
 *
 *  Created on: Jun 13, 2025
 *      Author: sunbeam
 */

#include<stm32f4xx.h>
#include"ultra.h"

#define trig      1
#define echo   0

void Ultra_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // TRIG CLOCK ENABLED
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // TIM2 CLOCK ENABLED

	// Here we initing the registers for the trig pin

	GPIOA->MODER |= BV(2*1+1);           //3RD BIT == 1
	GPIOA->MODER &= ~BV(2*1+2);       //4TH BIT == 0

	GPIOA->OTYPER &= ~BV(2);            //2ND BIT ==0

	GPIOA->OSPEEDR |= (3 << (2 * trig));
	//GPIOA->OSPEEDR |= BV(2*1+1) | BV(2*2+1);      //HIGH SPPED

	GPIOA->PUPDR &= ~(BV(2*1+1) | BV(2*1+2));  // NO-PULLUP PULLDOWN

	// Here we initing the registers for the echo pin

	GPIOA->MODER &= ~(3 << (2 * echo));     // Clear both bits
	GPIOA->MODER |=  (2 << (2 * echo));     // Set AF mode (10)
	//GPIOA->MODER |= BV(2*0+1);   // 1ST BIT == 1
	//GPIOA->MODER &= ~BV(2*0+0); // 0TH BIT == 0

	GPIOA->PUPDR &= ~(BV(2*0+0) | BV(2*0+1));  // NO-PULLUP PULLDOWN

	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;     // CC1 channel is input, mapped on TI1
	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S_1;

	TIM2->CCER &= ~TIM_CCER_CC1P;        // Initially capture rising edge
	TIM2->CCER |= TIM_CCER_CC1E;         // Enable capture on CH1



	//As we user echo pin as alternate function here we need to declare that echo pin as alternate function low...
	GPIOA->AFR[0] &= ~(0xF << (4 * echo));   // Clear
	GPIOA->AFR[0] |=  (0x1 << (4 * echo));   // AF1 = TIM2_CH1
	//GPIOA->AFR[0] |= BV(4*1+1);     //5TH BIT == 1
	//GPIOA->AFR[0] &= ~(BV(4*1+2) | BV(4*1+3) | BV(4*1+4));   //6,7,8 BIT == 0

	// Setting the timer registers
	TIM2->PSC = 83;
	TIM2->ARR = 0xFFFF;
	TIM2->CNT = 0;
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void Delay_us(uint32_t us)
{
	TIM2->CNT = 0;
	while(TIM2->CNT < us);
}
void Ultra_Trigger(void)
{
	GPIOA->BSRR = (1 << trig);           // HIGH
	Delay_us(10);
	GPIOA->BSRR = (1 << (trig + 16));    // LOW

    Delay_us(100);
	//GPIOA->BSRR = BV(17);
	//Delay_us(2);
	//GPIOA->BSRR = BV(1);
	//Delay_us(10);
	//GPIOA->BSRR = BV(17);
}
uint32_t Measure_Echo_Time(void)
{
	  // Disable capture while reconfiguring
	    TIM2->CCER &= ~TIM_CCER_CC1E;

	    // Set to capture rising edge
	    TIM2->CCER &= ~TIM_CCER_CC1P;
	    TIM2->SR &= ~TIM_SR_CC1IF;              // Clear flag
	    TIM2->CCER |= TIM_CCER_CC1E;            // Re-enable capture

	    while (!(TIM2->SR & TIM_SR_CC1IF));     // Wait for rising edge
	    uint32_t start = TIM2->CCR1;

	    // Disable capture again
	    TIM2->CCER &= ~TIM_CCER_CC1E;

	    // Set to capture falling edge
	    TIM2->CCER |= TIM_CCER_CC1P;
	    TIM2->SR &= ~TIM_SR_CC1IF;              // Clear flag
	    TIM2->CCER |= TIM_CCER_CC1E;

	    while (!(TIM2->SR & TIM_SR_CC1IF));     // Wait for falling edge
	    uint32_t end = TIM2->CCR1;

	    return (end >= start) ? (end - start) : (0xFFFF - start + end);

		//while((GPIOA->IDR & BV(0)) == 0);    // Wait untill echo pin is not high
	//TIM2->CNT = 0;                             // Echo pin high
	//while (GPIOA->IDR & BV(0));           // wait untill echo pin goes low
	//return TIM2->CNT;                         // Returning count when echo pin goes low

	// Here we calculated the time for which echo pin is high
}
uint32_t Get_Distance_Cm(void)
{
	Ultra_Trigger();
	    uint32_t time = Measure_Echo_Time();  // time in µs
	    return (time * 0.0343f) / 2;          // convert to cm
}
/*	int Calculate_Count(void)
	{
	    static int count = 0;
	    static uint8_t car_present = 0; // flag to avoid multiple counts

	    uint32_t distance = Get_Distance_Cm();

	    if(distance < 10 && car_present == 0) // Car just entered
	    {
	        count++;
	        car_present = 1;
	    }
	    else if(distance > 15 && car_present == 1) // Car moved away
	    {
	        car_present = 0;
	    }

	    return count;
	}
	*/

