/*
 * nrf2401.h
 *
 *  Created on: Nov 12, 2022
 *      Author: MSzozdowski
 */

#ifndef INC_NRF2401_NRF2401_H_
#define INC_NRF2401_NRF2401_H_

#define NRF_CE_LOW	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin, GPIO_PIN_RESET);
#define NRF_CE_HIGH	  	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin, GPIO_PIN_SET);

#define NRF_CSN_LOW	  	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
#define NRF_CSN_HIGH  	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

typedef enum
{
	NRF_NO_ERROR,
	NRF_NO_MODE,
	NRF_DIFFRENT_MESSAGE_SIZE,
	NRF_MAX_RETRANSMITS_FLAG,
	NRF_TX_FIFO_FULL
}NRF_Faults_t;

typedef enum{
	NRF_POWER_DOWN,
	NRF_START_UP,
	NRF_STANBY1,
	//NRF_STANBY2,
	NRF_RX_SETTING,
	NRF_TX_SETTING,
	NRF_TX_MODE,
	NRF_RX_MODE,
	NRF_IDLE
}NRF_State_t;

void NRF_Init(SPI_HandleTypeDef *hspi, char mode);
void NRF_process(uint8_t message);

#endif /* INC_NRF2401_NRF2401_H_ */
