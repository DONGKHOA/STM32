#ifndef DHT11_H_
#define DHT11_H_

#include "main.h"

#define START_TIME 18000

typedef struct 
{
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *GPIO_DHT11;
    uint16_t GPIO_Pin_DHT11;
    uint16_t start;
    float Hum;
    float Temp;
}DHT11_HandleTypeDef;

void DHT11_Init(DHT11_HandleTypeDef *DHT, TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIO_DHT11, uint16_t GPIO_Pin_DHT11);


#endif /* DHT11_H_ */
