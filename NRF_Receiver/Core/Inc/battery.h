/*
 * battery.h
 *
 *  Created on: Jan 31, 2023
 *      Author: Michal Szozdowski
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

void Battery_Init(ADC_HandleTypeDef* adc, uint32_t AdcChannel);
double Battery_GetVoltage();

#endif /* INC_BATTERY_H_ */
