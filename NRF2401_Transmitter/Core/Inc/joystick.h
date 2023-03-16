/*
 * joystick.h
 *
 *  Created on: Mar 4, 2023
 *      Author: Michal
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

typedef struct{
	uint32_t AdcChannel;
	ADC_HandleTypeDef adc;
} T_Joystick;

void Joystick_Init(T_Joystick* Joystick, ADC_HandleTypeDef* adc ,uint32_t AdcChannel);
void Joystick_SetChannel(T_Joystick* Joystick);
uint16_t Joystick_GetValue(T_Joystick* Joystick);
uint8_t Joystick_Map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#endif /* INC_JOYSTICK_H_ */
