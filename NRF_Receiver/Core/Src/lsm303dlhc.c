/*
 * lsm303dlhc.c
 *
 *  Created on: Jan 31, 2023
 *      Author: Michal Szozdowski
 */

#include "main.h"
#include "lsm303dlhc.h"

I2C_HandleTypeDef* sens_i2c;

// Functions to write and read data from LSM303DLHC accelerometer and magnetometer
static HAL_StatusTypeDef LSM303DLHC_Write_Acc(uint8_t address, uint8_t data)
{
  return HAL_I2C_Mem_Write(sens_i2c, LSM303DLHC_ACC_ADDRESS, address, 1, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef LSM303DLHC_Read_Acc(uint8_t address, uint8_t *data)
{
  return HAL_I2C_Mem_Read(sens_i2c, LSM303DLHC_ACC_ADDRESS, address, 1, data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef LSM303DLHC_Write_Mag(uint8_t address, uint8_t data)
{
  return HAL_I2C_Mem_Write(sens_i2c, LSM303DLHC_MAG_ADDRESS, address, 1, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef LSM303DLHC_Read_Mag(uint8_t address, uint8_t *data)
{
  return HAL_I2C_Mem_Read(sens_i2c, LSM303DLHC_MAG_ADDRESS, address, 1, data, 1, HAL_MAX_DELAY);
}

// Function to initialize LSM303DLHC accelerometer and magnetometer
void LSM303DLHC_Init(I2C_HandleTypeDef* hi2c)
{
  sens_i2c = hi2c;
  // Initialize accelerometer
  LSM303DLHC_Write_Acc(0x20, 0x27);
  LSM303DLHC_Write_Acc(0x23, 0x00);

  // Initialize magnetometer
  LSM303DLHC_Write_Mag(0x00, 0x00);
  LSM303DLHC_Write_Mag(0x02, 0x00);
}

// Function to read accelerometer data
void LSM303DLHC_Read_Acc_Data(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t data[6];

  LSM303DLHC_Read_Acc(0x28, &data[0]);
  LSM303DLHC_Read_Acc(0x29, &data[1]);
  LSM303DLHC_Read_Acc(0x2A, &data[2]);
  LSM303DLHC_Read_Acc(0x2B, &data[3]);
  LSM303DLHC_Read_Acc(0x2C, &data[4]);
  LSM303DLHC_Read_Acc(0x2D, &data[5]);

  *x = (int16_t)((data[1] << 8) | data[0]);
  *y = (int16_t)((data[3] << 8) | data[2]);
  *z = (int16_t)((data[5] << 8) | data[4]);
}

// Function to read magnetometer data
void LSM303DLHC_Read_Mag_Data(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t data[6];

  LSM303DLHC_Read_Mag(0x03, &data[0]);
  LSM303DLHC_Read_Mag(0x04, &data[1]);
  LSM303DLHC_Read_Mag(0x05, &data[2]);
  LSM303DLHC_Read_Mag(0x06, &data[3]);
  LSM303DLHC_Read_Mag(0x07, &data[4]);
  LSM303DLHC_Read_Mag(0x08, &data[5]);

  *x = (int16_t)((data[1] << 8) | data[0]);
  *y = (int16_t)((data[3] << 8) | data[2]);
  *z = (int16_t)((data[5] << 8) | data[4]);
}
