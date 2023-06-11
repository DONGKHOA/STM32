/*
 * button.c
 *
 *  Created on: May 29, 2023
 *      Author: khoa dong
 */


#include "button.h"

__weak void BUTTON_Pressing_Callback(BUTTON_HandleTypeDef *ButtonX)
{
}

__weak void BUTTON_Releasing_Callback(BUTTON_HandleTypeDef *ButtonX)
{
}

/**
 * This function initializes the button by setting the GPIO port and pin, the current state, the last
 * state, the debouncing state, the debouncing flag, and the debouncing timer
 *
 * @param ButtonX The button you want to initialize
 * @param GPIO_Button The GPIO port that the button is connected to.
 * @param GPIO_Pin_Button The pin number of the button.
 */
void BUTTON_Init(BUTTON_HandleTypeDef *ButtonX, GPIO_TypeDef *GPIO_Button, uint16_t GPIO_Pin_Button)
{
	ButtonX->GPIO_Button = GPIO_Button;
	ButtonX->GPIO_Pin_Button = GPIO_Pin_Button;
	ButtonX->current_state = 1;
	ButtonX->last_state = 1;
	ButtonX->deboucing_state = 1;
	ButtonX->is_deboucing = 0;
	ButtonX->deboucing_timer = 0;
}

/**
 * If the button is pressed, call the pressing callback function, if the button is released, call the
 * releasing callback function.
 *
 * @param ButtonX The button you want to handle.
 */
void BUTTON_Handle(BUTTON_HandleTypeDef *ButtonX)
{
	// detecting
	uint8_t temp_state = HAL_GPIO_ReadPin(ButtonX->GPIO_Button, ButtonX->GPIO_Pin_Button);
	if (temp_state != ButtonX->deboucing_state)
	{
		ButtonX->deboucing_state = temp_state;
		ButtonX->deboucing_timer = HAL_GetTick();
		ButtonX->is_deboucing = 1;
	}

	// deboucing
	if (ButtonX->is_deboucing == 1 && (HAL_GetTick() - ButtonX->deboucing_timer) > 15)
	{
		ButtonX->current_state = ButtonX->deboucing_state;
		ButtonX->is_deboucing = 0;
	}

	if (ButtonX->current_state != ButtonX->last_state)
	{
		if (ButtonX->current_state == 0) // Press the button
		{
			BUTTON_Pressing_Callback(ButtonX);
		}
		else // Release button
		{
			BUTTON_Releasing_Callback(ButtonX);
		}
		ButtonX->last_state = ButtonX->current_state;
	}
}
