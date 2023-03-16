/*
 * button.c
 *
 *  Created on: Jan 29, 2023
 *      Author: Michal Szozdowski
 */

#include "main.h"
#include "button.h"

#define DEBOUNCE_TIME 20

static GPIO_PinState Button_ReadPin(T_Button *Button)
{
	return HAL_GPIO_ReadPin(Button->GpioPort, Button->GpioPin);
}

void Button_Init(T_Button *Button, GPIO_TypeDef *GpioPort, uint16_t GpioPin)
{
	Button->state = IDLE;
	Button->GpioPort = GpioPort;
	Button->GpioPin = GpioPin;
}

void Button_Process(T_Button *Button)
{
	switch(Button->state)
	{
	case IDLE:
		if(Button_ReadPin(Button) == GPIO_PIN_RESET)
		{
			Button->state = DEBOUNCE;
			Button->last_tick = HAL_GetTick();
		}
		break;
	case DEBOUNCE:
		if(HAL_GetTick() - (Button->last_tick) > DEBOUNCE_TIME)
		{
			if(Button_ReadPin(Button) == GPIO_PIN_RESET)
				Button->state = PRESSED;
			else
				Button->state = IDLE;
		}
		break;
	case PRESSED:
		if(Button_ReadPin(Button) == GPIO_PIN_SET)
			Button->state = IDLE;
		break;
	}
}

uint8_t Button_IsPressed(T_Button *Button)
{
	if((Button->state) == PRESSED)
		return 1;
	else
		return 0;
}