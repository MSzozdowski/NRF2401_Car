/*
 * button.h
 *
 *  Created on: Mar 4, 2023
 *      Author: stasz
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include <stdbool.h>

typedef enum
{
	IDLE,
	DEBOUNCE,
	PRESSED
}BUTTON_STATE;

typedef struct
{
	BUTTON_STATE 	state;

	GPIO_TypeDef 	*GpioPort;
	uint16_t 		GpioPin;

	uint32_t 		last_tick;
	bool switch_status;
}T_Button;

void Button_Init(T_Button *Button, GPIO_TypeDef *GpioPort, uint16_t GpioPin);
void Button_Process(T_Button *Button);
uint8_t Button_GetSwitchStatus(T_Button *Button);

#endif /* INC_BUTTON_H_ */
