/*
 * lsm303dlhc.h
 *
 *  Created on: Jan 31, 2023
 *      Author: Michal Szozdowski
 */

#ifndef INC_LSM303DLHC_H_
#define INC_LSM303DLHC_H_

#define LSM303DLHC_ACC_ADDRESS   0x19
#define LSM303DLHC_MAG_ADDRESS   0x1E

void LSM303DLHC_Init(I2C_HandleTypeDef* hi2c);
void LSM303DLHC_Read_Acc_Data(int16_t *x, int16_t *y, int16_t *z);
void LSM303DLHC_Read_Mag_Data(int16_t *x, int16_t *y, int16_t *z);

#endif /* INC_LSM303DLHC_H_ */
