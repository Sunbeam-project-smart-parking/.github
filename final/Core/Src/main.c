/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include "I2C.h"
#include "LCD.h"
#include "string.h"
#include "uart.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t flag0 =0;
volatile uint8_t flag1 =0;
volatile uint8_t flag2 =0;
volatile uint8_t flag3 =0;
volatile uint8_t flag4 =0;
volatile uint8_t flag5 =0;
volatile uint8_t flag8 =0;
int car_Slot =2;
int bus_Slot =2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_Servo_Angle(TIM_HandleTypeDef *htim,uint32_t channel,uint8_t angle)
{
	uint32_t PULSE_SET = 210 + (angle *(1050-210)/180);
	__HAL_TIM_SET_COMPARE(htim,channel,PULSE_SET);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    switch(GPIO_Pin)
       {
           case GPIO_PIN_0:
              if(flag0 == 0)
               flag0 =1;
               Set_Servo_Angle(&htim3, TIM_CHANNEL_3, 90);
               break;

           case GPIO_PIN_1:
        	   if(flag1 ==0)
        	   flag1 =1;
        	   Set_Servo_Angle(&htim3, TIM_CHANNEL_4, 90);
               break;

           case GPIO_PIN_2:
               if(flag2==0)
               flag2 =1;
               if(bus_Slot >0)
               Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 90);
               break;

           case GPIO_PIN_3:
               if(flag3==0)
               flag3=1;
               Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 90);
               break;

           case GPIO_PIN_4:
        	   if(flag4 == 0)
        	   flag4 =1;
        	   if(car_Slot >0)
        	   Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 90);
               break;

           case GPIO_PIN_5:
        	   if(flag5 ==0)
        	   flag5 =1;
        	   Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 90);
               break;
           case GPIO_PIN_8:
           if(flag8 ==0)
           flag8 =1;
           Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 90);
           Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 90);
           Set_Servo_Angle(&htim3, TIM_CHANNEL_3, 90);
           Set_Servo_Angle(&htim3, TIM_CHANNEL_4, 90);
           break;


           default:
               break;
       }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char msg[32];
	char str[20];
	uint8_t car_in =0;
	uint8_t car_out =0;
	uint8_t bus_in =0;
	uint8_t bus_out =0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  char main_scr[20];

  	    I2CInit();
  	    HAL_Delay(100);

  	    UartInit(11500);


  	    LcdInit(SLOT_1);
  	    HAL_Delay(100);
  	    LcdPuts(SLOT_1, LCD_LINE1, "LCD1 OK!");
  	    LcdPuts(SLOT_1,LCD_LINE2, "Tested");

  	    LcdInit(SLOT_2);
  	    HAL_Delay(100);
  	    LcdPuts(SLOT_2, LCD_LINE1, "LCD2 OK!");
  	    LcdPuts(SLOT_2, LCD_LINE2, "Tested");


  	    LcdInit(MAIN);
  	    HAL_Delay(100);
  	    LcdPuts(MAIN, LCD_LINE1, "MAIN_LCD OK!");
  	    LcdPuts(MAIN, LCD_LINE2, "Tested");
  	    LcdPuts(MAIN, LCD_LINE3, "Tested");
  	    LcdPuts(MAIN, LCD_LINE4, "Tested");

  	  LcdPuts(MAIN, LCD_LINE1, "--------------------");
  	  LcdPuts(MAIN, LCD_LINE2, "|      WELCOME     |");
  	  LcdPuts(MAIN, LCD_LINE3, "|   SUNBEAM SMART  |");
  	  LcdPuts(MAIN, LCD_LINE4, "|      PARKING     |");
      LcdPuts(SLOT_1,LCD_LINE1,"  Smart Parking ");
  	  LcdPuts(SLOT_1,LCD_LINE2,"     Sunbeam   ");
  	  HAL_Delay(3000);

      LcdPuts(SLOT_2,LCD_LINE1,"  Smart Parking ");
  	  LcdPuts(SLOT_2,LCD_LINE2,"     Sunbeam   ");
  	  HAL_Delay(3000);

  	  Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 0);
  	  Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 0);
      Set_Servo_Angle(&htim3, TIM_CHANNEL_3, 0);
      Set_Servo_Angle(&htim3, TIM_CHANNEL_4, 0);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //----------------node mcu transer--------------


	     UartPuts("DESD @ Sunbeam.\r\n");
	     sprintf(msg,"car_slot = %d ,bus_slot = %d \r\n",car_Slot,bus_Slot);
	     UartPuts(msg);


	  	  //-----------------main_lcd-----------------
	  	  LcdPuts(MAIN, LCD_LINE1, "--------------------");
	  	  LcdPuts(MAIN, LCD_LINE2, "| CAR-PARK BUS-PARK|");
	  	  //LcdPuts(MAIN, LCD_LINE3, "|TOTAL  2   TOTAL  2 |");
	  	  sprintf(main_scr,"| AVAIL-%d   AVAIL-%d|",car_Slot,bus_Slot);
	  	  LcdPuts(MAIN, LCD_LINE3,main_scr);
	  	  LcdPuts(MAIN, LCD_LINE4,"--------------------");
	  	  //-----------------alert--------------------
	  	  if(flag8 ==1)
	  	  {
	  		  LcdPuts(SLOT_1,LCD_LINE1,"____ALERT !!____");
	  		  LcdPuts(SLOT_1,LCD_LINE2,"___FIRE ALARM___");
	  		  LcdPuts(SLOT_2,LCD_LINE1,"____ALERT !!____");
	  		  LcdPuts(SLOT_2,LCD_LINE2,"___FIRE ALARM___");
	  		  while(1)
	  		  {
	  			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET);
	  			  HAL_Delay(200);
	  			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
	  			  HAL_Delay(200);

	  		  }
	  	  }
	  	  //----------------main_gate------------------
	  	  if(flag0== 1)
	  	  {
	  		  HAL_Delay(1000);
	  		  Set_Servo_Angle(&htim3, TIM_CHANNEL_3, 0);
	  		  flag0 =0;
	  	  }
	  	  if(flag1== 1)
	  	  {
	  	  	  Set_Servo_Angle(&htim3, TIM_CHANNEL_4, 0);
	  	  	  flag1 =0;
	  	  }

	  	  //--------------slot ---------------------------

	  	  LcdPuts(SLOT_1,LCD_LINE1,"____BUS SLOT____" );

	  	 	  if(bus_Slot > 0 )
	  	 	  {
	  	 	  sprintf(str ," AVAILABLE =  %d \r\n",bus_Slot);
	  	 	  LcdPuts(SLOT_1,LCD_LINE2, str);

	  	 	  if(flag2 == 1)
	  	 	  	  {


	  	 	  	                Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 90);
	  	 	  	                HAL_Delay(2000);
	  	 	  	                bus_in =1;

	  	 	  	                while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)== GPIO_PIN_SET))
	  	 	  	                {
	  	 	  	                	HAL_Delay(2000);

	  	 	  	                }

	  	 	  	            // Sweep back from 90 to 0 degrees

	  	 	  	                Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 0);
	  	 	  	               // HAL_Delay(2000);
	  	 	  	                flag2 = 0;

	  	 		  	  	  	    if(flag3==1  && bus_in ==1)
	  	 		  	  	  		  {
	  	 		  	  	  			  bus_Slot--;

	  	 		  	  	  			  flag3 =0;
	  	 		  	  	  			  bus_in =0;

	  	 		  	  	  		  }



	  	 	  	  }
	  	 	  else if(flag3 == 1)
	  	 	  {
	  	 		  bus_label:
	  	 		  Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 90);
	  	 		 	  	                HAL_Delay(2000);
	  	 		 	  	                bus_out =1;

	  	 		 	  	                while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== GPIO_PIN_SET))
	  	 		 	  	                {
	  	 		 	  	                	HAL_Delay(2000);

	  	 		 	  	                }

	  	 		 	  	            // Sweep back from 90 to 0 degrees

	  	 		 	  	                Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 0);
	  	 		 	  	               // HAL_Delay(2000);
	  	 		 	  	                flag3 = 0;

	  	 		 		  	  	  	    if(flag2==1  && bus_out ==1)
	  	 		 		  	  	  		  {
	  	 		 		  	  	  			  bus_Slot++;

	  	 		 		  	  	  			  flag2 =0;
	  	 		 		  	  	  			  bus_out =0;

	  	 		 		  	  	  		  }


	  	 	  }


	  	 	  }
	  	 	  else
	  	 	  {
	  	 		  LcdPuts(SLOT_1, LCD_LINE2, "FULL !          \r");

	  	 		  if(flag3 == 1)
	  	 			  goto bus_label;
	  	 	  }



	  	 	  /////////////////////////////////////////////////////////////

	  	 	 LcdPuts(SLOT_2,LCD_LINE1,"____CAR SLOT____" );

	  	 	 	 	  if(car_Slot > 0 )
	  	 	 	 	  {
	  	 	 	 	  sprintf(str ," AVAILABLE =  %d \r\n",car_Slot);
	  	 	 	 	  LcdPuts(SLOT_2,LCD_LINE2, str);

	  	 	 	 	  if(flag4 == 1)
	  	 	 	 	  	  {


	  	 	 	 	  	                Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 90);
	  	 	 	 	  	                HAL_Delay(2000);
	  	 	 	 	  	                car_in =1;

	  	 	 	 	  	                while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)== GPIO_PIN_SET))
	  	 	 	 	  	                {
	  	 	 	 	  	                	HAL_Delay(2000);

	  	 	 	 	  	                }

	  	 	 	 	  	            // Sweep back from 90 to 0 degrees

	  	 	 	 	  	                Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 0);
	  	 	 	 	  	               // HAL_Delay(2000);
	  	 	 	 	  	                flag4 = 0;

	  	 	 	 		  	  	  	    if(flag5==1  && car_in ==1)
	  	 	 	 		  	  	  		  {
	  	 	 	 		  	  	  			  car_Slot--;

	  	 	 	 		  	  	  			  flag5 =0;
	  	 	 	 		  	  	  			  car_in =0;

	  	 	 	 		  	  	  		  }



	  	 	 	 	  	  }
	  	 	 	 	  else if(flag5 == 1)
	  	 	 	 	  {
	  	 	 	 		  car_label:
	  	 	 	 		  Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 90);
	  	 	 	 		 	  	                HAL_Delay(2000);
	  	 	 	 		 	  	                car_out =1;

	  	 	 	 		 	  	                while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)== GPIO_PIN_SET))
	  	 	 	 		 	  	                {
	  	 	 	 		 	  	                	HAL_Delay(2000);

	  	 	 	 		 	  	                }

	  	 	 	 		 	  	            // Sweep back from 90 to 0 degrees

	  	 	 	 		 	  	                Set_Servo_Angle(&htim3, TIM_CHANNEL_2, 0);
	  	 	 	 		 	  	               // HAL_Delay(2000);
	  	 	 	 		 	  	                flag5 = 0;

	  	 	 	 		 		  	  	  	    if(flag4==1  && car_out ==1)
	  	 	 	 		 		  	  	  		  {
	  	 	 	 		 		  	  	  			  car_Slot++;

	  	 	 	 		 		  	  	  			  flag4 =0;
	  	 	 	 		 		  	  	  			  car_out =0;

	  	 	 	 		 		  	  	  		  }


	  	 	 	 	  }


	  	 	 	 	  }
	  	 	 	 	  else
	  	 	 	 	  {
	  	 	 	 		  LcdPuts(SLOT_2, LCD_LINE2, "FULL !          \r");

	  	 	 	 		  if(flag5 == 1)
	  	 	 	 			  goto car_label;
	  	 	 	 	  }



    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8400-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ULTRA_TRIGGER_GPIO_Port, ULTRA_TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MAIN_IR_1_Pin MAIN_IR_2_Pin SLOT_1_IR_Pin SLOT_1_IRA3_Pin
                           SLOT_2_IR_Pin SLOT_2_IRA5_Pin GAS_SENSOR_Pin */
  GPIO_InitStruct.Pin = MAIN_IR_1_Pin|MAIN_IR_2_Pin|SLOT_1_IR_Pin|SLOT_1_IRA3_Pin
                          |SLOT_2_IR_Pin|SLOT_2_IRA5_Pin|GAS_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULTRA_ECO_Pin */
  GPIO_InitStruct.Pin = ULTRA_ECO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ULTRA_ECO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULTRA_TRIGGER_Pin */
  GPIO_InitStruct.Pin = ULTRA_TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ULTRA_TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
