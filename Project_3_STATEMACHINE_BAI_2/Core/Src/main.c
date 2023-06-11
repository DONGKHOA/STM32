/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE BEGIN PV */
enum{
	effect_0,
	effect_1,
	effect_2,
};
//button var
uint8_t current_state = 1;
uint8_t last_state = 1;
uint8_t deboucing_state = 1;
uint8_t is_deboucing = 0;
uint32_t deboucing_timer = 0;
uint8_t is_pressing = 0;
uint32_t pressing_timer = 0;

//led var
uint8_t effect = effect_0;
uint8_t off = 0;
uint32_t led_timer = 0;
uint32_t temp_timer = 0;
#define LED_1 0x00000078
#define LED_2 0x00000018
#define LED_3 0x00000060
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BUTTON_Handel(uint16_t gpio_pin)
{
	// detecting
	uint8_t temp_state = HAL_GPIO_ReadPin(GPIOA, gpio_pin);
	if(temp_state != deboucing_state)
	{
		deboucing_state = temp_state;
		deboucing_timer = HAL_GetTick();
		is_deboucing = 1;
	}

	// deboucing
	if(is_deboucing==1 && (HAL_GetTick() - deboucing_timer > 15))
	{
		current_state = deboucing_state;
		is_deboucing = 0;
	}

	if(last_state != current_state)
	{
		if(current_state==0)
		{
			is_pressing = 1;
			pressing_timer = HAL_GetTick();
		}
		else
		{
			is_pressing = 0;
			if(HAL_GetTick() - pressing_timer < 2000)
			{
				effect++;
				if(effect > effect_2)
				{
					effect = effect_0;
				}
			}
		}
		last_state = current_state;
	}
	if(is_pressing == 1 && (HAL_GetTick() - pressing_timer > 2000))
	{
		off = !off;
		is_pressing = 0;
	}
}
void LED_Handel()
{
	if(off == 0)
	{
		switch(effect)
		{
		case effect_0:
			if(HAL_GetTick() - led_timer > 400)
			{
				if((GPIOB->ODR & LED_1) == 0)
				{
					GPIOB->ODR = GPIOB->ODR | LED_1;
				}
				else
				{
					GPIOB->ODR = GPIOB->ODR & ~LED_1;
				}
				led_timer = HAL_GetTick();
			}
			break;
		case effect_1:
			if(HAL_GetTick() - led_timer > 400)
			{
				if((GPIOB->ODR & LED_2) == 0)
				{
					GPIOB->ODR = GPIOB->ODR | LED_2;
					GPIOB->ODR = GPIOB->ODR & ~LED_3;
				}
				else
				{
					GPIOB->ODR = GPIOB->ODR & ~LED_2;
					GPIOB->ODR = GPIOB->ODR | LED_3;
				}
				led_timer = HAL_GetTick();
			}
			break;
		case effect_2:

			if(HAL_GetTick() - temp_timer >= 1000)
			{
				led_timer = HAL_GetTick();
				temp_timer = HAL_GetTick();
			}

			if(HAL_GetTick() - led_timer < 200)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			}
			else if( HAL_GetTick() - led_timer < 400)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			}
			else if( HAL_GetTick() - led_timer < 600)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			}
			else if(HAL_GetTick() - led_timer > 600)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
			}
			break;
		}
	}
	else
	{
		GPIOB->ODR = GPIOB->ODR & ~LED_1;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  BUTTON_Handel(GPIO_PIN_0);
	  LED_Handel();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
