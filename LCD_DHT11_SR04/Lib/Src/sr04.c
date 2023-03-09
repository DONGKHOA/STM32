#include "sr04.h"
#include "delay_timer.h"

/**
 * The function is used to measure the distance between the sensor and the object in front of it
 * 
 * @param htim pointer to the timer that will be used to measure the time
 * @param GPIO_Trig The GPIO port that the trigger pin is connected to.
 * @param GPIO_Pin_Trig the pin number of the trigger pin
 * @param GPIO_Echo The GPIO port where the echo pin is connected.
 * @param GPIO_Pin_Echo The pin number of the echo pin
 * 
 * @return The distance in cm.
 */
uint8_t GET_Distance(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIO_Trig, uint16_t GPIO_Pin_Trig, GPIO_TypeDef* GPIO_Echo, uint16_t GPIO_Pin_Echo)
{
    HAL_GPIO_WritePin(GPIO_Trig, GPIO_Pin_Trig, 1);
    DELAY_Tim_Us(htim, 10);

    HAL_TIM_Base_Stop(htim);
    HAL_GPIO_WritePin(GPIO_Trig, GPIO_Pin_Trig, 0);


    while(HAL_GPIO_ReadPin(GPIO_Echo,GPIO_Pin_Echo) == 0);

    HAL_TIM_Base_Start(htim);

    while (1)
    {
        if (HAL_GPIO_ReadPin(GPIO_Echo,GPIO_Pin_Echo) == 0)
        {
            HAL_TIM_Base_Stop(htim);
            break;
        }
		if((htim -> Instance -> CNT) > MAX_DISTANCE)
		{
			HAL_TIM_Base_Stop(htim);
			return 0;
		}
    }
    return 0.017 * (htim -> Instance -> CNT);   //unit of measure: cm
}
