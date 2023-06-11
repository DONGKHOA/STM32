#include "sr04.h"
#include "delay_timer.h"

#define MAX_DISTANCE 171600
#define TIME_OUT 2000 // ms

static void SR04_Delay_Init(SR04_HandleTypeDef *SR_04x)
{
    DELAY_Tim_Init(SR_04x->htim);
}

static void SR04_Delay_Us(SR04_HandleTypeDef *SR_04x, uint16_t time_us)
{
    DELAY_Tim_Us(SR_04x->htim, time_us);
}

/**
 * SR04_Init() initializes the SR04_HandleTypeDef structure, which is a handle to the SR04 sensor
 *
 * @param SR_04x is the handle for the sensor
 * @param htim pointer to the timer used for the delay
 * @param GPIO_Trig The GPIO port that the trigger pin is connected to.
 * @param GPIO_Pin_Trig The pin number of the GPIO port that the trigger pin is connected to.
 * @param GPIO_Echo The GPIO port that the echo pin is connected to.
 * @param GPIO_Pin_Echo The pin that the echo signal is connected to.
 */
void SR04_Init(SR04_HandleTypeDef *SR_04x, TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIO_Trig, uint16_t GPIO_Pin_Trig, GPIO_TypeDef *GPIO_Echo, uint16_t GPIO_Pin_Echo)
{
    SR_04x->htim = htim;
    SR_04x->GPIO_Echo = GPIO_Echo;
    SR_04x->GPIO_Trig = GPIO_Trig;
    SR_04x->GPIO_Pin_Echo = GPIO_Pin_Echo;
    SR_04x->GPIO_Pin_Trig = GPIO_Pin_Trig;
    SR04_Delay_Init(SR_04x);
}

/**
 * The function sends a 10us pulse to the trigger pin of the sensor, then waits for the echo pin to go
 * high. When it does, it starts the timer. When the echo pin goes low, it stops the timer and returns
 * the distance in cm
 *
 * @param SR_04x is the handle of the sensor
 *
 * @return The distance in cm.
 */
uint8_t SR04_Get_Distance(SR04_HandleTypeDef *SR_04x)
{
    uint32_t timer = 0;
    HAL_GPIO_WritePin(SR_04x->GPIO_Trig, SR_04x->GPIO_Pin_Trig, 1);
    SR04_Delay_Us(SR_04x, 10);

    HAL_TIM_Base_Stop(SR_04x->htim);
    HAL_GPIO_WritePin(SR_04x->GPIO_Trig, SR_04x->GPIO_Pin_Trig, 0);

    while (HAL_GPIO_ReadPin(SR_04x->GPIO_Echo, SR_04x->GPIO_Pin_Echo) == 0)
    {
        if ((HAL_GetTick() - timer) > TIME_OUT)
        {
            return 0;
        }
    }
    HAL_TIM_Base_Start(SR_04x->htim);

    while (1)
    {
        if ((HAL_GetTick() - timer) > TIME_OUT)
        {
            return 0;
        }

        if (HAL_GPIO_ReadPin(SR_04x->GPIO_Echo, SR_04x->GPIO_Pin_Echo) == 0)
        {
            HAL_TIM_Base_Stop(SR_04x->htim);
            break;
        }

        if ((SR_04x->htim->Instance->CNT) > MAX_DISTANCE)
        {
            HAL_TIM_Base_Stop(SR_04x->htim);
            return 0;
        }
    }
    return 0.017 * (SR_04x->htim->Instance->CNT); // unit of measure: cm
}
