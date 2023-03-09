#ifndef BUTTON_H_
#define BUTTON_H_

#include "stdio.h"

uint8_t current_state = 1;
uint8_t last_state = 1;
uint8_t deboucing_state = 1;
uint8_t is_deboucing = 0;
uint32_t deboucing_timer = 0;
uint8_t condition = 0;

void BUTTON_Handel(GPIO_TypeDef *GPIO_Button, uint16_t GPIO_Pin_Button);

#endif /* BUTTON_H_ */
