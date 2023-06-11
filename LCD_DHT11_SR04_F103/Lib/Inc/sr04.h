#ifndef SR04_H_
#define SR04_H_

#include "main.h"

typedef struct 
{
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *GPIO_Trig;
    uint16_t GPIO_Pin_Trig;
    GPIO_TypeDef *GPIO_Echo;
    uint16_t GPIO_Pin_Echo;
}SR04_HandleTypeDef;

void SR04_Init(SR04_HandleTypeDef *SR_04x, TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIO_Trig, uint16_t GPIO_Pin_Trig, GPIO_TypeDef *GPIO_Echo, uint16_t GPIO_Pin_Echo);

uint8_t SR04_Get_Distance(SR04_HandleTypeDef *SR_04x);

#endif /* SR04_H_ */
