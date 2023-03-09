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

/**
 * "This function takes a pointer to a timer handle and a time in microseconds and sets the timer
 * counter to zero and then waits until the timer counter is greater than the time in microseconds."
 * 
 * The first line of the function sets the timer counter to zero. The second line is a while loop that
 * waits until the timer counter is greater than the time in microseconds
 * 
 * @param htim pointer to the timer handle
 * @param time the time in microseconds
 */
void DELAY_Tim_Us(TIM_HandleTypeDef *htim, uint16_t time_us)
{
	htim -> Instance -> CNT = 0;
	while((htim -> Instance -> CNT ) < time);
}

/**
 * The function takes a pointer to a timer handle and a time in milliseconds. It sets the counter to
 * zero and then waits for the counter to reach the time in milliseconds.
 * 
 * @param htim pointer to the timer handle
 * @param time the time in milliseconds you want to delay
 */
void DELAY_Tim_Ms(TIM_HandleTypeDef *htim, uint16_t time_ms)
{
	htim -> Instance -> CNT = 0;
	while(time--)
	{
		while((htim -> Instance -> CNT ) < time);
	}

}
