/*
 * drv8835.h
 *
 *  Created on: Jan 15, 2023
 *      Author: Michal
 */

#ifndef INC_DRV8835_H_
#define INC_DRV8835_H_

#define DRV8835_FORWARD 0
#define DRV8835_REVERSE 1

#define DRV8835_BRAKE	2

#define DRV8835_PHASE_MODE 0
#define DRV8835_IN_MODE 1

void DRV8835_Init(TIM_HandleTypeDef *htim, uint64_t right_channel_pwm_timer, uint64_t left_channel_pwm_timer);
void DRV8835_RunRightMotor(uint8_t direction, uint8_t speed);
void DRV8835_RunLeftMotor(uint8_t direction, uint8_t speed);

#endif /* INC_DRV8835_H_ */
