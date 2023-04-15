/*
 * clock.c
 *
 *  Created on: Nov 26, 2022
 *      Author: MSzozdowski
 */

#include "main.h"
#include "clock.h"

static uint32_t tick; //100us tickrate

TIM_HandleTypeDef *clock_tim;

void Clock_Init(TIM_HandleTypeDef *htim)
{
	clock_tim = htim;
	HAL_TIM_Base_Start_IT(htim);
	tick = 0;
}

uint32_t Clock_GetTick()
{
	return tick;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == clock_tim)
	{
		tick+=100;
	}
}
