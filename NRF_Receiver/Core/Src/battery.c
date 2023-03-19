/*
 * battery.c
 *
 *  Created on: Jan 31, 2023
 *      Author: Michal Szozdowski
 */
#include "main.h"
#include "battery.h"
#include "adc.h"

#define REFERENCE_VOLTAGE 3.3
#define SCALE 2

ADC_HandleTypeDef* bat_adc;
uint32_t BatAdcChannel;

void Battery_Init(ADC_HandleTypeDef* adc, uint32_t AdcChannel)
{
	bat_adc = adc;
	BatAdcChannel = AdcChannel;
}

static void Battery_SetChannel(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}


double Battery_GetVoltage(void)
{
	double adc_value;
	Battery_SetChannel();
	HAL_ADC_Start(bat_adc);
		if(HAL_ADC_PollForConversion(bat_adc, 1000) == HAL_OK)
		{
			adc_value = HAL_ADC_GetValue(bat_adc);
			return ((adc_value/4095)*REFERENCE_VOLTAGE*SCALE);
		}
		return 0;
}
//return (((HAL_ADC_GetValue(bat_adc))/4095)*2);
