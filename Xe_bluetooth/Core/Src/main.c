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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t Rx_Buff;
uint8_t UART_buffer[1000];
uint8_t UART_num = 0;
uint32_t time = 0;
uint32_t time_pressing = 0;
volatile int16_t en_counter_1 = 0, en_counter_2 = 0, pre_en_counter_1 = 0, pre_encounter_2 = 0;
volatile float rate_1 = 0, rate_2 = 0, rate_60_1 = 0, rate_60_2 = 0;
float Kp1 = 49, Kp2 = 63;
float Ki1 = 0, Ki2 = 0;
float Kd1 = 40, Kd2 = 56;
float PID_val_1, pre_PID_val_1 = 0;
float PID_val_2, pre_PID_val_2 = 0;
float error_1 = 0, pre_error_1 = 0, pre_pre_error_1 = 0, error_2 = 0, pre_error_2 = 0, pre_pre_error_2 = 0;
float first_val = 280;
float P_1 = 0, P_2 = 0;
float I_1 = 0, I_2 = 0;
float D_1 = 0, D_2 = 0;
uint8_t s = 30;
int PID_1 = 0, PID_2 = 0;
int PWM1 = 0, PWM2 = 0;
int condition = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us){
	htim4.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim4);
	while(htim4.Instance->CNT < us);
	HAL_TIM_Base_Stop(&htim4);
}

uint8_t get_distance()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);

	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 0);

	HAL_TIM_Base_Start(&htim4);

	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 0)
		{
			HAL_TIM_Base_Stop(&htim4);
			break;
		}
		if(htim4.Instance->CNT > 171600)
		{
			HAL_TIM_Base_Stop(&htim4);
			return 0;
		}
	}
	return 0.017 * htim4.Instance->CNT;
}

void cal_pid()
{
	if(HAL_GetTick() - time >= 1000)
	{
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		rate_1 =(en_counter_1 - pre_en_counter_1)/374.0;
		rate_60_1 = rate_1 * 15;
		rate_2 =(en_counter_2 - pre_encounter_2)/374.0;
		rate_60_2 = rate_2 * 15;

		pre_en_counter_1 = en_counter_1;
		pre_encounter_2 = en_counter_2;

		error_1 = first_val - rate_60_1;
		error_2 = first_val - rate_60_2;

		time = HAL_GetTick();
	}
	en_counter_2 = __HAL_TIM_GET_COUNTER(&htim3);
	en_counter_1 = __HAL_TIM_GET_COUNTER(&htim2);

	P_1 = error_1;
	I_1 = pre_error_1 + error_1;
	D_1 = error_1 - 2*pre_error_1 + pre_pre_error_2;
	PID_val_1 = (Kp1 * P_1) + (Ki1 *0.5 * I_1) + (Kd1 *D_1);
	pre_pre_error_1 = pre_error_1;
	pre_error_1 = error_1;
	pre_PID_val_1 = PID_val_1;
	P_2 = error_2;
	I_2 = pre_error_2 + error_2;
	D_2 = error_2 - 2*pre_error_2 + pre_pre_error_2;
	PID_val_2 = (Kp2 * P_2) + (Ki2 * I_2) + (Kd2 *D_2);
	pre_pre_error_1 = pre_error_1;
	pre_error_2 = error_2;
	pre_PID_val_2 = PID_val_2;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		if(UART_num == 0)
		{
			UART_buffer[UART_num] = Rx_Buff;
			UART_num++;
		}
		else
		{
			UART_buffer[UART_num] = Rx_Buff;
			if(UART_buffer[UART_num] == UART_buffer[UART_num - 1])
			{
				UART_num++;
			}
			else
			{
				UART_num = 0;
			}
		}
		HAL_UART_Receive_IT(huart, &Rx_Buff, 1);
	}
}
void ham_chuyen()
{
	if(error_1  < 0)
	{
		PID_1 = first_val - PID_val_1;
	}
	else
	{
		PID_1 = first_val + PID_val_1;
	}
	if(error_2 < 0)
	{
		PID_2 = first_val - PID_val_2;
	}
	else
	{
		PID_2 = first_val + PID_val_2;
	}
	PWM1 = (PID_1 + 86.602)/0.3664;
	PWM2 = (PID_2 + 84.177)/0.3664;

}
void UART_Handle()
{
	if(get_distance() <= s)
	{
		condition = 0;
	}
	else
	{
		condition = 1;
	}
	if (condition == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		switch((char)UART_buffer[0])
					{
					case 'F':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
							cal_pid();
							ham_chuyen();
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
						break;
					case 'B':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,960);
						break;
					case 'L':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
						break;
					case 'R':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
						break;

					case 'I':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,700);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
						break;
					case 'J':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,700);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
						break;
					case 'G':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,700);
						break;
					case 'H':
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,700);
						break;
					default:
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
						break;
					}
	}
	if (condition ==1)
	{
 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
		switch((char)UART_buffer[0])
			{
			case 'F':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					cal_pid();
					ham_chuyen();
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM2);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM1);


				break;
			case 'B':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,960);
				break;
			case 'L':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
				break;
			case 'R':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
				break;

			case 'I':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,700);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
				break;
			case 'J':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,700);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
				break;
			case 'G':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,700);
				break;
			case 'H':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,700);
				break;
			default:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				break;
			}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &Rx_Buff, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UART_Handle();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart3.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
