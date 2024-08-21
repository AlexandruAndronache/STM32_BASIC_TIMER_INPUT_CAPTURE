/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#define TRUE 1
#define FALSE 0
#define SYS_CLOCK_FREQ_50_MHZ 50
#define SYS_CLOCK_FREQ_80_MHZ 80
#define SYS_CLOCK_FREQ_120_MHZ 120
//#include "cmsis_os.h"
//#include "usart.h"
//#include "gpio.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(uint8_t clock_freq);
void MX_FREERTOS_Init(void);
void GPIO_INIT(void);
void Error_Handler(void);
void TIM2_Init();
void LSE_Configuration(void);

TIM_HandleTypeDef htimer2;
uint8_t FLatency = 0;
uint32_t rcc_frequency = 0;
int main(void)
{

 HAL_Init();

  SystemClock_Config(SYS_CLOCK_FREQ_80_MHZ);

  GPIO_INIT();

  TIM2_Init();

  //LSE_Configuration();

  while (1)
  {
	  rcc_frequency = HAL_RCC_GetHCLKFreq();
	  HAL_Delay(1000);
  }

  return 0;

}

void LSE_Configuration(void)
{
	RCC_OscInitTypeDef osc_init;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	osc_init.LSEState = RCC_LSE_ON;

	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}


	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

}

void TIM2_Init()
{
	TIM_IC_InitTypeDef timer2IC_Config;

	timer2IC_Config.ICPolarity = TIM_ICPOLARITY_RISING; //Active edge
	timer2IC_Config.ICFilter = 0; //Filter for the signal
	timer2IC_Config.ICPrescaler = TIM_ICPSC_DIV1; //Capture performed each time an edge is detected on the capture input
	timer2IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI; //input channel selection
	if(HAL_TIM_IC_ConfigChannel(&htimer2, &timer2IC_Config, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	htimer2.Instance = TIM2;
	htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htimer2.Init.Period = 0xFFFF;
	htimer2.Init.Prescaler = 1;
	htimer2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htimer2.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	if(HAL_TIM_IC_Init(&htimer2) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_RCC_TIM2_CLK_ENABLE();
}

void GPIO_INIT()
{
	//GPIO_TypeDef gpio;

	GPIO_InitTypeDef gpio_handle;

	gpio_handle.Pin = GPIO_PIN_5;
	gpio_handle.Mode = GPIO_MODE_AF_PP;
	gpio_handle.Pull = GPIO_NOPULL;
	gpio_handle.Speed = GPIO_SPEED_FREQ_MEDIUM;



	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &gpio_handle);
	//HAL_GPIO_WritePin();

	//Let's start the timer
	HAL_TIM_Base_Start(&htimer2);




}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(uint8_t clock_freq)
{
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
	osc_init.HSIState = RCC_HSI_ON;
	osc_init.LSEState = RCC_LSE_ON;
	osc_init.HSICalibrationValue = 15;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	switch(clock_freq)
	{
	case SYS_CLOCK_FREQ_50_MHZ:
	{
		osc_init.PLL.PLLM = 16;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLR = 2;
		osc_init.PLL.PLLP = 2;

	    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
	    					RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
	    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
	    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

	    FLatency = FLASH_ACR_LATENCY_1WS;;
		break;
	}

	case SYS_CLOCK_FREQ_80_MHZ:
	{

		osc_init.PLL.PLLM = 16;
		osc_init.PLL.PLLN = 160;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLR = 2;
		osc_init.PLL.PLLP = 2;

	    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
	    					RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
	    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
	    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

	    FLatency = FLASH_ACR_LATENCY_2WS;
		break;
	}

	case SYS_CLOCK_FREQ_120_MHZ:
	{
		osc_init.PLL.PLLM = 16;
		osc_init.PLL.PLLN = 240;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLR = 2;
		osc_init.PLL.PLLP = 4;

	    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
	    					RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
	    clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
	    clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

	    FLatency = FLASH_ACR_LATENCY_3WS;
		break;
	}

	default:
		return;
	}



	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
		{
		Error_Handler();
		}

	if(HAL_RCC_ClockConfig( &clk_init, FLatency) != HAL_OK)
		{
		Error_Handler();
		}

	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

	//Systick frequency
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


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
