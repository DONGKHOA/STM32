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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// declare UART
uint8_t Rx_Buff = 0;
uint8_t uart_buffer[100];
uint8_t position_uart = 0;
uint8_t uart_flag = 0;

// declare effect led
enum{
	EFFECT_0,
	EFFECT_1,
	EFFECT_2,
	EFFECT_OFF,
};
uint8_t effect = EFFECT_0;
uint8_t ON = 0;
uint32_t led_timer = 0;
uint32_t temp_timer = 0;
#define LED_1 0x00000078
#define LED_2 0x00000018
#define LED_3 0x00000060

// declare button
uint8_t deboucing_state = 1;
uint8_t current_state = 1;
uint8_t last_state = 1;
uint8_t is_deboucing = 0;
uint8_t is_pressing = 0;
uint32_t deboucing_timer = 0;
uint32_t pressing_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		if(Rx_Buff != '\n')
		{
			uart_buffer[position_uart] = Rx_Buff;
			position_uart++;
		}
		else
		{
			uart_buffer[position_uart] = '\0';
			uart_flag = 1;
		}
		HAL_UART_Receive_IT(huart, &Rx_Buff, 1);
	}
}
void UART_Handle()
{
	if(uart_flag==1)
	{
		//cut string

		char *arg_list[10];
		uint8_t arg_position = 0;

		char *temp_token = strtok((char *)uart_buffer, " ");
		while(temp_token != NULL)
		{
			arg_list[arg_position]= temp_token;
			arg_position++;
			temp_token = strtok(NULL, " ");
		}

		// Solution
		if(strcmp(arg_list[0], "EFFECT")==0)
		{
			if(strcmp(arg_list[1], "0")==0) effect = EFFECT_0;
			if(strcmp(arg_list[1], "1")==0) effect = EFFECT_1;
			if(strcmp(arg_list[1], "2")==0) effect = EFFECT_2;
		}
		if(strcmp(arg_list[0], "OFF")==0)
		{
			effect = EFFECT_OFF;
		}
		uart_flag = 0;
		position_uart = 0;
	}
}

void BUTTON_Handle()
{
	uint8_t temp_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	// detecting

	if(temp_state != deboucing_state)
	{
		deboucing_state = temp_state;
		is_deboucing = 1;
		deboucing_timer = HAL_GetTick();
	}

	//deboucing

	if(is_deboucing == 1 && (HAL_GetTick() - deboucing_timer > 15))
	{
		current_state = deboucing_state;
		is_deboucing = 0;
	}
	if(current_state != last_state)
	{
		if(current_state==0)
		{
			is_pressing = 1;
			pressing_timer = HAL_GetTick();
		}
		else
		{
			is_pressing = 0;
		}
		last_state = current_state;
	}
	if(is_pressing == 1 && (HAL_GetTick() - pressing_timer > 2000))
	{
		ON = !ON;
		is_pressing = 0;
	}
}

void LED_Effect()
{
	if(ON==1)
	{
		switch(effect)
		{
		case EFFECT_0:
			if(HAL_GetTick() - led_timer > 500)
			{
				if((GPIOB->ODR & LED_1)==0)
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
		case EFFECT_1:
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
		case EFFECT_2:
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
		case EFFECT_OFF:
			GPIOB->ODR = GPIOB->ODR & ~LED_1;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &Rx_Buff, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UART_Handle();
	  BUTTON_Handle();
	  LED_Effect();
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
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
