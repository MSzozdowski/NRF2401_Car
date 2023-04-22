/*
 * button.c
 *
 *  Created on: Jan 29, 2023
 *      Author: Michal Szozdowski
 */

#include "main.h"
#include "button.h"
#include "stdio.h"


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
	Button->switch_status = 0;
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
			{
				Button->state = PRESSED;
				Button->switch_status = !(Button->switch_status);
			}

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

uint8_t Button_GetSwitchStatus(T_Button *Button)
{
	return (uint8_t)Button->switch_status;
}
