#ifndef SR04_H_
#define SR04_H_

#include "main.h"

#define MAX_DISTANCE 171600

uint8_t GET_Distance(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIO_Trig, uint16_t GPIO_Pin_Trig, GPIO_TypeDef* GPIO_Echo, uint16_t GPIO_Pin_Echo);
#endif /* SR04_H_ */
