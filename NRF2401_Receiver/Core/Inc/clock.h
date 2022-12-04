/*
 * clock.h
 *
 *  Created on: Nov 26, 2022
 *      Author: MSzozdowski
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

void Clock_Init(TIM_HandleTypeDef *htim);
uint32_t Clock_GetTick();

#endif /* INC_CLOCK_H_ */
