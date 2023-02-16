/*
 * lights.c
 *
 *  Created on: Jan 29, 2023
 *      Author: Michal Szozdowski
 */

#include "main.h"
#include "lights.h"

void Lights_SetState(GPIO_TypeDef* GpioPort, uint16_t GpioPin, uint8_t lights_state)
{
	GPIO_PinState pin_state;
	if(lights_state)
		pin_state = GPIO_PIN_SET;
	else
		pin_state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(GpioPort, GpioPin, pin_state);
}
