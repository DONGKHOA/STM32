#include "dht11.h"
#include "delay_timer.h"

static void DHT11_Delay_Init(DHT11_HandleTypeDef *DHT)
{
    DELAY_Tim_Init(DHT->htim);
}

static void DHT11_Delay_Us(DHT11_HandleTypeDef *DHT, uint16_t time_us)
{
    DELAY_Tim_Us(DHT->htim, time_us);
}

/**
 * It initializes the DHT11_HandleTypeDef structure with the start time, GPIO port, GPIO pin, and timer
 * handle
 * 
 * @param DHT pointer to the DHT11_HandleTypeDef structure
 * @param htim TIM_HandleTypeDef *htim;
 * @param GPIO_DHT11 The GPIO port that the DHT11 is connected to.
 * @param GPIO_Pin_DHT11 The pin that the DHT11 is connected to.
 */
void DHT11_Init(DHT11_HandleTypeDef *DHT, TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIO_DHT11, uint16_t GPIO_Pin_DHT11)
{
    DHT->start = START_TIME;
    DHT->GPIO_DHT11 = GPIO_DHT11;
    DHT->GPIO_Pin_DHT11 = GPIO_Pin_DHT11;
    DHT->htim = htim;
    DHT11_Delay_Init(DHT);
}

static uint8_t DHT11_Start(DHT11_HandleTypeDef *DHT)
{
    
}