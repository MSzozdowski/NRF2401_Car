/*
 * lsm303dlhc.c
 *
 *  Created on: Mar 19, 2023
 *      Author: Michal2018
 */

#include "main.h"
#include "lsm303dlhc.h"
#include <limits.h>

I2C_HandleTypeDef *LSM_i2c;

float Gvalue_x;
float Gvalue_y;
float Gvalue_z;

static void LSM303DLHC_WriteRegister(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(LSM_i2c, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, 1, &value, 1, LSM303DLHC_TIMEOUT);
}

static uint8_t LSM303DLHC_ReadRegister(uint8_t reg)
{
	uint8_t value;
	HAL_I2C_Mem_Read(LSM_i2c, ACC_I2C_ADDRESS, reg, 1, &value, 1, LSM303DLHC_TIMEOUT);
	return value;
}

static int16_t LSM303DLHC_ReadTwoRegisters(uint8_t reg)
{
	uint8_t reg_values[2];
	HAL_I2C_Mem_Read(LSM_i2c, ACC_I2C_ADDRESS, (reg | 0x80), 1, reg_values, 2, LSM303DLHC_TIMEOUT); //0x80 autoincrement of address
	return (reg_values[1] << 8 | reg_values[0]);
}

void LSM303DLHC_Init(I2C_HandleTypeDef *hi2c2)
{
	LSM_i2c = hi2c2;
	LSM303DLHC_WriteRegister(LSM303DLHC_CTRL_REG1_A, LSM303DLHC_X_ENABLE | LSM303DLHC_Y_ENABLE | LSM303DLHC_Z_ENABLE | LSM303DLHC_ODR_10_HZ);
}

void LSM303DLHC_ReadValues(float *value_x_g, float *value_y_g, float *value_z_g)
{
	int16_t value_x = LSM303DLHC_ReadTwoRegisters(LSM303DLHC_OUT_X_L_A);
	int16_t value_y = LSM303DLHC_ReadTwoRegisters(LSM303DLHC_OUT_Y_L_A);
	int16_t value_z = LSM303DLHC_ReadTwoRegisters(LSM303DLHC_OUT_Z_L_A);

	*value_x_g = ((float) value_x * LSM303DLHC_ACC_RESOLUTION_2G * GRAVITATIONAL_ACCELERATION) / (float) INT16_MAX;
	*value_y_g = ((float) value_y * LSM303DLHC_ACC_RESOLUTION_2G * GRAVITATIONAL_ACCELERATION) / (float) INT16_MAX;
	*value_z_g = ((float) value_z * LSM303DLHC_ACC_RESOLUTION_2G * GRAVITATIONAL_ACCELERATION) / (float) INT16_MAX;

	//round values
	//*value_x_g = ((int)(*value_x_g * 100 + .5) / 100.0);
	//*value_y_g = ((int)(*value_y_g * 100 + .5) / 100.0);
	//*value_z_g = ((int)(*value_z_g * 100 + .5) / 100.0);
}
