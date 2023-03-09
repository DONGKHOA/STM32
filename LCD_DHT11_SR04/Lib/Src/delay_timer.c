#include "delay_timer.h"

/**
 * This function initializes the timer used for the delay function
 * 
 * @param htim pointer to the timer handle
 */
void DELAY_Tim_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Start(htim);
}

void DELAY_Tim_Us(TIM_HandleTypeDef *htim, uint16_t time_us)
{
	HAL_TIM_Base_Start(htim);
	htim -> Instance -> CNT = 0;
	while((htim -> Instance -> CNT ) < time_us);
	HAL_TIM_Base_Stop(htim);
}

void DELAY_Tim_Ms(TIM_HandleTypeDef *htim, uint16_t time_ms)
{
	while(time_ms--)
	{
		DELAY_Tim_Us(htim, 1000);
	}
}
