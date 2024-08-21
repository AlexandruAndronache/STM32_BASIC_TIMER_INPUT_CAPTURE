
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */
 void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{

	 GPIO_InitTypeDef tim2ch1_gpio;

	 //1. Enable clock for tim2
	 __HAL_RCC_TIM2_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	 //2. COnfigure a pin as timer2 channel1
	 tim2ch1_gpio.Pin = GPIO_PIN_0;
	 tim2ch1_gpio.Mode = GPIO_MODE_AF_PP;
	 tim2ch1_gpio.Alternate = GPIO_AF1_TIM2;
	 HAL_GPIO_Init(GPIOA, &tim2ch1_gpio);

	 //3. HAL NVIC settings
	 HAL_NVIC_SetPriority(TIM2_IRQn, 15, 0);
	 HAL_NVIC_EnableIRQ(TIM2_IRQn);

}

/* USER CODE END 1 */
