/*
 * nrf2401.h
 *
 *  Created on: Nov 12, 2022
 *      Author: MSzozdowski
 */

#ifndef INC_NRF2401_NRF2401_H_
#define INC_NRF2401_NRF2401_H_

#include "main.h"

#define NRF_CSN_HIGH	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)
#define NRF_CSN_LOW		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)

#define NRF_CE_HIGH		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)
#define NRF_CE_LOW		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)

void NRF_Init(SPI_HandleTypeDef *hspi);

//
// READ/WRITE REGISTERS
//
uint8_t NRF_ReadConfigRegister();
void NRF_WriteConfig(uint8_t config);
uint8_t NRF_ReadStatus(void);
void NRF_WriteStatus(uint8_t status);

//
// SWITCHING BETWEEN RX AND TX
//
void NRF_RX_Mode(void);
void NRF_TX_Mode(void);

//
// RADIO SETTINGS
//
void NRF_SetOutputPower(uint8_t output_power);
void NRF_SetDataRate(uint8_t data_rate);
void NRF_SetCRC(uint8_t crc_en, uint8_t crc_length);
void NRF_SetRetransmission(uint8_t retransmission_delay, uint8_t retranmission_repeat);
void NRF_SetChannel(uint8_t channel);
void NRF_SetPayloadSize(uint8_t pipe_number, uint8_t size);

void NRF_SetDataPipe(uint8_t pipe_number);
void NRF_EnableAutoACK(uint8_t pipe_number, uint8_t enable);

void NRF_SetRXAddress(uint8_t pipe, uint8_t* address); // Remember to define RX address before TX
void NRF_SetTXAddress(uint8_t* address);
void NRF_SetAddressWidth(uint8_t address_width);

//
// PUSH/PULL DATA TO PAYLOAD
//
void NRF_TXPayload(uint8_t *data, uint8_t size);
void NRF_ReadRXPaylaod(uint8_t *data);

//
// FLUSHING FIFOs
//
void NRF_FlushRX(void);
void NRF_FlushTX(void);

//
// POLLING METHOD
//
uint8_t NRF_DataAvailable(void);

#endif /* INC_NRF2401_NRF2401_H_ */
