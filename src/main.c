/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ws2812b_multi_strip_driver.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId blinkTaskHandle;
osSemaphoreId semHandle;
extern uint8_t LED_strips[MAX_SUPPORTED_NUM_OF_STRIPS][MAX_SUPPORTED_LEDS_IN_STRIP][NUM_OF_CFG_BYTES_PER_LED];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void BlinkTask(void const * argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(blinkTask, BlinkTask, osPriorityRealtime, 0, 128);
  blinkTaskHandle = osThreadCreate(osThread(blinkTask), NULL);
  osSemaphoreDef(sem);
  semHandle = osSemaphoreCreate(osSemaphore(sem), 1);
  osSemaphoreWait(semHandle, osWaitForever);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC6 PC7 
                           PC8 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA6 
                           PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB13 PB14 PB15 
                           PB4 PB5 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void disableAllIRQs(void)
{
	/******  STM32 specific Interrupt Numbers **********************************************************************/
	  NVIC_DisableIRQ(WWDG_IRQn              );
	  NVIC_DisableIRQ(PVD_IRQn               );
	  NVIC_DisableIRQ(TAMP_STAMP_IRQn        );
	  NVIC_DisableIRQ(RTC_WKUP_IRQn          );
	  NVIC_DisableIRQ(FLASH_IRQn             );
	  NVIC_DisableIRQ(RCC_IRQn               );
	  NVIC_DisableIRQ(EXTI0_IRQn             );
	  NVIC_DisableIRQ(EXTI1_IRQn             );
	  NVIC_DisableIRQ(EXTI2_IRQn             );
	  NVIC_DisableIRQ(EXTI3_IRQn             );
	  NVIC_DisableIRQ(EXTI4_IRQn             );
	  NVIC_DisableIRQ(DMA1_Stream0_IRQn      );
	  NVIC_DisableIRQ(DMA1_Stream1_IRQn      );
	  NVIC_DisableIRQ(DMA1_Stream2_IRQn      );
	  NVIC_DisableIRQ(DMA1_Stream3_IRQn      );
	  NVIC_DisableIRQ(DMA1_Stream4_IRQn      );
	  NVIC_DisableIRQ(DMA1_Stream5_IRQn      );
	  NVIC_DisableIRQ(DMA1_Stream6_IRQn      );
	  NVIC_DisableIRQ(ADC_IRQn               );
	  NVIC_DisableIRQ(CAN1_TX_IRQn           );
	  NVIC_DisableIRQ(CAN1_RX0_IRQn          );
	  NVIC_DisableIRQ(CAN1_RX1_IRQn          );
	  NVIC_DisableIRQ(CAN1_SCE_IRQn          );
	  NVIC_DisableIRQ(EXTI9_5_IRQn           );
	  NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn     );
	  NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn     );
	  NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	  NVIC_DisableIRQ(TIM1_CC_IRQn           );
	  NVIC_DisableIRQ(TIM2_IRQn              );
	  NVIC_DisableIRQ(TIM3_IRQn              );
	  NVIC_DisableIRQ(TIM4_IRQn              );
	  NVIC_DisableIRQ(I2C1_EV_IRQn           );
	  NVIC_DisableIRQ(I2C1_ER_IRQn           );
	  NVIC_DisableIRQ(I2C2_EV_IRQn           );
	  NVIC_DisableIRQ(I2C2_ER_IRQn           );
	  NVIC_DisableIRQ(SPI1_IRQn              );
	  NVIC_DisableIRQ(SPI2_IRQn              );
	  NVIC_DisableIRQ(USART1_IRQn            );
	  NVIC_DisableIRQ(USART2_IRQn            );
	  NVIC_DisableIRQ(USART3_IRQn            );
	  NVIC_DisableIRQ(EXTI15_10_IRQn         );
	  NVIC_DisableIRQ(RTC_Alarm_IRQn         );
	  NVIC_DisableIRQ(OTG_FS_WKUP_IRQn       );
	  NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn    );
	  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn     );
	  NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	  NVIC_DisableIRQ(TIM8_CC_IRQn           );
	  NVIC_DisableIRQ(DMA1_Stream7_IRQn      );
	  NVIC_DisableIRQ(FMC_IRQn               );
	  NVIC_DisableIRQ(SDIO_IRQn              );
	  NVIC_DisableIRQ(TIM5_IRQn              );
	  NVIC_DisableIRQ(SPI3_IRQn              );
	  NVIC_DisableIRQ(UART4_IRQn             );
	  NVIC_DisableIRQ(UART5_IRQn             );
	  NVIC_DisableIRQ(TIM6_DAC_IRQn          );
	  NVIC_DisableIRQ(TIM7_IRQn              );
	  NVIC_DisableIRQ(DMA2_Stream0_IRQn      );
	  NVIC_DisableIRQ(DMA2_Stream1_IRQn      );
	  NVIC_DisableIRQ(DMA2_Stream2_IRQn      );
	  NVIC_DisableIRQ(DMA2_Stream3_IRQn      );
	  NVIC_DisableIRQ(DMA2_Stream4_IRQn      );
	  NVIC_DisableIRQ(CAN2_TX_IRQn           );
	  NVIC_DisableIRQ(CAN2_RX0_IRQn          );
	  NVIC_DisableIRQ(CAN2_RX1_IRQn          );
	  NVIC_DisableIRQ(CAN2_SCE_IRQn          );
	  NVIC_DisableIRQ(OTG_FS_IRQn            );
	  NVIC_DisableIRQ(DMA2_Stream5_IRQn      );
	  NVIC_DisableIRQ(DMA2_Stream6_IRQn      );
	  NVIC_DisableIRQ(DMA2_Stream7_IRQn      );
	  NVIC_DisableIRQ(USART6_IRQn            );
	  NVIC_DisableIRQ(I2C3_EV_IRQn           );
	  NVIC_DisableIRQ(I2C3_ER_IRQn           );
	  NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn    );
	  NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn     );
	  NVIC_DisableIRQ(OTG_HS_WKUP_IRQn       );
	  NVIC_DisableIRQ(OTG_HS_IRQn            );
	  NVIC_DisableIRQ(DCMI_IRQn              );
	  NVIC_DisableIRQ(FPU_IRQn               );
	  NVIC_DisableIRQ(SPI4_IRQn              );
	  NVIC_DisableIRQ(SAI1_IRQn              );
	  NVIC_DisableIRQ(SAI2_IRQn              );
	  NVIC_DisableIRQ(QUADSPI_IRQn           );
	  NVIC_DisableIRQ(CEC_IRQn               );
	  NVIC_DisableIRQ(SPDIF_RX_IRQn          );
	  NVIC_DisableIRQ(FMPI2C1_EV_IRQn        );
	  NVIC_DisableIRQ(FMPI2C1_ER_IRQn        );
}

void BlinkTask(void const *argument)
{
	volatile int idx;
	volatile int i,dummy;

	if (osSemaphoreWait(semHandle, osWaitForever) == osOK)
	{
		//disableAllIRQs();
		LD2_GPIO_Port->ODR |= LD2_Pin;
		for (i=0; i<MAX_LEDS_IN_STRIP; i++)
		{
			LED_strips[0][i][0] = 250;
			LED_strips[0][i][1] = 250;
			LED_strips[0][i][2] = 250;
			LED_strips[1][i][0] = 255;
			LED_strips[1][i][1] = 0;
			LED_strips[1][i][2] = 0;
			LED_strips[2][i][0] = 0;
			LED_strips[2][i][1] = 255;
			LED_strips[2][i][2] = 0;
			LED_strips[3][i][0] = 0;
			LED_strips[3][i][1] = 0;
			LED_strips[3][i][2] = 255;
			LED_strips[4][i][0] = 200;
			LED_strips[4][i][1] = 0;
			LED_strips[4][i][2] = 200;
			LED_strips[5][i][0] = 200;
			LED_strips[5][i][1] = 100;
			LED_strips[5][i][2] = 0;
			LED_strips[6][i][0] = 0;
			LED_strips[6][i][1] = 100;
			LED_strips[6][i][2] = 200;

		}
		for (;;)
		{
			update_GPIO_all_strips_mask(GPIO_PIN_10);
			update_driver_mask(0);
			drive_port_strips();
			osDelay(100);
			update_driver_mask(1);
			drive_port_strips();
			osDelay(100);
			update_driver_mask(2);
			drive_port_strips();
			osDelay(100);
			update_driver_mask(3);
			drive_port_strips();
			osDelay(100);
			update_driver_mask(4);
			drive_port_strips();
			osDelay(100);
			update_driver_mask(5);
			drive_port_strips();
			osDelay(100);
			update_driver_mask(6);
			drive_port_strips();
			osDelay(100);


			//GPIOA->ODR |= GPIO_PIN_10;
			/*for (idx=0; idx < 16; idx++)
			{
				//Configure 0
				GPIOA->ODR |=  GPIO_PIN_10;
				for (i=0; i < 5; i++) {dummy=i;}
				GPIOA->ODR &= ~GPIO_PIN_10;
				for (i=0; i < 15; i++) {dummy=i;}
			}
			for (idx=0; idx < 8; idx++)
			{
				//configure 1
				GPIOA->ODR |=  GPIO_PIN_10;
				for (i=0; i < 15; i++) {dummy=i;}
				GPIOA->ODR &= ~GPIO_PIN_10;
				for (i=0; i < 5; i++) {dummy=i;}
			}*/
			//Configure 0
/*			GPIOA->ODR |=  GPIO_PIN_10;
			for (i=0; i < 10; i++) {idx=i;}
			GPIOA->ODR &= ~GPIO_PIN_10;
			for (i=0; i < 30; i++) {idx=i;}*/
			//idx=1;

		}
		osThreadTerminate(NULL);
	}
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	  {
		  osSemaphoreRelease(semHandle);
		  osThreadTerminate(NULL);
	  }
    //osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
