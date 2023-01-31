/*
 * drv8835.c
 *
 *  Created on: Jan 15, 2023
 *      Author: Michal
 */

#include "main.h"
#include "drv8835.h"
#include "tim.h"

TIM_HandleTypeDef *pwm_tim;

static uint64_t right_channel_timer;
static uint64_t left_channel_timer;

static void DRV8835_SetMode(uint8_t mode);
static void DRV8835_SetRightMotorDirection(uint8_t direction);
static void DRV8835_SetLeftMotorDirection(uint8_t direction);

void DRV8835_Init(TIM_HandleTypeDef *htim, uint64_t right_channel_pwm_timer, uint64_t left_channel_pwm_timer)
{
	pwm_tim = htim;
	right_channel_timer = right_channel_pwm_timer;
	left_channel_timer = left_channel_pwm_timer;

	DRV8835_SetMode(DRV8835_PHASE_MODE);
	HAL_TIM_PWM_Start(pwm_tim, right_channel_pwm_timer);
	HAL_TIM_PWM_Start(pwm_tim, left_channel_pwm_timer);
}

static void DRV8835_SetMode(uint8_t mode)
{
	if(mode == DRV8835_PHASE_MODE)
		HAL_GPIO_WritePin(DRV_MODE_GPIO_Port, DRV_MODE_Pin, SET);
	if(mode == DRV8835_IN_MODE)
		HAL_GPIO_WritePin(DRV_MODE_GPIO_Port, DRV_MODE_Pin, RESET);
}

static void DRV8835_SetRightMotorDirection(uint8_t direction)
{
	if(direction == DRV8835_FORWARD)
		HAL_GPIO_WritePin(A_PHASE_GPIO_Port, A_PHASE_Pin, RESET);
	if(direction == DRV8835_BACKWARD)
		HAL_GPIO_WritePin(A_PHASE_GPIO_Port, A_PHASE_Pin, SET);
}

static void DRV8835_SetLeftMotorDirection(uint8_t direction)
{
	if(direction == DRV8835_FORWARD)
		HAL_GPIO_WritePin(B_PHASE_GPIO_Port, B_PHASE_Pin, RESET);
	if(direction == DRV8835_BACKWARD)
		HAL_GPIO_WritePin(B_PHASE_GPIO_Port, B_PHASE_Pin, SET);
}

static void DRV8835_RunRightMotor(uint8_t direction, uint8_t speed)
{
	DRV8835_SetRightMotorDirection(direction);
	if(speed > htim1.Init.Period) //should be set to 128
		speed = htim1.Init.Period;
	__HAL_TIM_SET_COMPARE(pwm_tim, right_channel_timer, speed);
}

static void DRV8835_RunLeftMotor(uint8_t direction, uint8_t speed)
{
	DRV8835_SetLeftMotorDirection(direction);
	if(speed > htim3.Init.Period) //should be set to 128
		speed = htim3.Init.Period;
	__HAL_TIM_SET_COMPARE(pwm_tim, left_channel_timer, speed);
}

void DRV8835_Move(uint8_t direction, uint8_t veer)
{
	if(direction > 128) //forward
	{
		if(veer > 128) //forward right
		{

			DRV8835_RunRightMotor(DRV8835_FORWARD, direction - 128 - (veer-128)/2);
			DRV8835_RunLeftMotor(DRV8835_FORWARD, direction - 128);
		}
		else if(veer < 128) //forward left
		{

			DRV8835_RunRightMotor(DRV8835_FORWARD, direction - 128);
			DRV8835_RunLeftMotor(DRV8835_FORWARD, direction - 128 - (veer-128)/2);
		}
		else //forward
		{

			DRV8835_RunRightMotor(DRV8835_FORWARD, direction - 128);
			DRV8835_RunLeftMotor(DRV8835_FORWARD, direction - 128);
		}
	}
	else if(direction < 128) //backward
	{
		if(veer > 128) //backward right
		{
			DRV8835_RunRightMotor(DRV8835_BACKWARD, 128 - direction - (veer-128)/2);
			DRV8835_RunLeftMotor(DRV8835_BACKWARD, 128 - direction);
		}
		else if(veer < 128)
		{
			DRV8835_RunRightMotor(DRV8835_BACKWARD, 128 - direction);
			DRV8835_RunLeftMotor(DRV8835_BACKWARD, 128 - direction - (veer-128)/2);
		}
		else
		{
			DRV8835_RunRightMotor(DRV8835_BACKWARD, 128 - direction);
			DRV8835_RunLeftMotor(DRV8835_BACKWARD, 128 - direction);
		}
	}
	else //stay
	{
		if(veer > 128)
		{
			DRV8835_RunRightMotor(DRV8835_FORWARD, 0);
			DRV8835_RunLeftMotor(DRV8835_FORWARD, veer - 128);
		}
		else if(veer < 128)
		{
			DRV8835_RunRightMotor(DRV8835_FORWARD, veer - 128);
			DRV8835_RunLeftMotor(DRV8835_FORWARD, 0);
		}
		else
		{
			DRV8835_RunRightMotor(DRV8835_FORWARD, 0);
			DRV8835_RunLeftMotor(DRV8835_FORWARD, 0);
		}
	}
}
