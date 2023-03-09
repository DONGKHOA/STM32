#include "button.h"

/**
 * It's a function that debounces a button
 * 
 * @param GPIO_Button The GPIO port that the button is connected to.
 * @param GPIO_Pin_Button The pin that the button is connected to.
 */
void BUTTON_Handel(GPIO_TypeDef *GPIO_Button, uint16_t GPIO_Pin_Button)
{
	// detecting
	uint8_t temp_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if(temp_state != deboucing_state)
	{
		deboucing_state = temp_state;
		deboucing_timer = HAL_GetTick();
		is_deboucing = 1;
	}

	// deboucing
	if(is_deboucing == 1 && (HAL_GetTick() - deboucing_timer) > 15)
	{
		current_state = deboucing_state;
		is_deboucing = 0;
	}

	if(current_state != last_state)
	{
		if(current_state == 0)  // Press the button
		{
			condition = 1;
		}
		else                    // Release button
		{
			condition = 0;
		}
		last_state = current_state;
	}
}