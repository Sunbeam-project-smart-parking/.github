/*
 * ultra.h
 *
 *  Created on: Jun 13, 2025
 *      Author: sunbeam
 */

#ifndef ULTRA_H_
#define ULTRA_H_

#include<stm32f4xx.h>

void Ultra_Init(void);
void Ultra_Trigger(void);
void Delay_us(uint32_t us);
uint32_t Measure_Echo_Time(void);
uint32_t Get_Distance_Cm(void);
int Calculate_Count(void);




#endif /* ULTRA_H_ */
