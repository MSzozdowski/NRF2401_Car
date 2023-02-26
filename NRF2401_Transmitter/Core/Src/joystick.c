/*
 * joystick.c
 *
 *  Created on: Jan 21, 2023
 *      Author: Michal
 */

#include "main.h"
#include "joystick.h"
#include "adc.h"

void Joystick_Init(T_Joystick* Joystick, ADC_HandleTypeDef* adc ,uint32_t AdcChannel)
{
	Joystick->adc = *adc;
	Joystick->AdcChannel = AdcChannel;
}

void Joystick_SetChannel(T_Joystick* Joystick)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

uint16_t Joystick_GetValue(T_Joystick* Joystick)
{
	Joystick_SetChannel(Joystick);
	HAL_ADC_Start(&Joystick -> adc);
	if(HAL_ADC_PollForConversion(&Joystick -> adc, 1000) == HAL_OK)
		return Joystick_Map(HAL_ADC_GetValue(&Joystick -> adc), 0, 4095, 0, 255);
	return 0;
}

uint8_t Joystick_Map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

