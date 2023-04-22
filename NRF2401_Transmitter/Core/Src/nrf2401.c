/*
 * nrf2401.c
 *
 *  Created on: Nov 12, 2022
 *      Author: MSzozdowski
 */

#include "main.h"
#include "NRF2401/nrf2401.h"
#include "NRF2401/nrf2401_defs.h"
#include "clock.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

SPI_HandleTypeDef *NRF_spi;

NRF_State_t NRF_State = NRF_POWER_DOWN;
NRF_Faults_t NRF_Faults = NRF_NO_ERROR;

static uint8_t tx_buffer[NRF24_PAYLOAD_SIZE /*+ 1*/];
static uint8_t rx_buffer[NRF24_PAYLOAD_SIZE];

static uint8_t message_length;
static bool message_transmit = false;

static uint32_t lastTick100us, last_tick, message_send_tick;

uint8_t message_send_counter = 0;

static char nrf_mode;

static uint8_t addr_p0_backup[NRF24_ADD_WIDTH];

static void NRF_CalculateBitrate(void);
static void NRF_Read(uint8_t *Data, uint8_t Length);
static void NRF_Write(uint8_t *Data, uint8_t Length);

static uint8_t NRF_ReadRegister(uint8_t reg);
static void NRF_ReadRegisters(uint8_t reg, uint8_t* data, uint8_t length);

static void NRF_WriteRegister(uint8_t reg, uint8_t data);
static void NRF_WriteRegisters(uint8_t reg, uint8_t* data, uint8_t length);

static void NRF_SendCommand(uint8_t command);

static uint8_t NRF_ReadConfigRegister(void);
static void NRF_WriteConfigRegister(uint8_t config);

static uint8_t NRF_ReadStatusRegister(void);
static void NRF_WriteStatusRegister(uint8_t status);

static void NRF_SetOutputPower(uint8_t output_power);
static void NRF_SetDataRate(uint8_t data_rate);
static void NRF_SetCRC(uint8_t crc_en, uint8_t crc_length);
static void NRF_SetRetransmission(uint8_t retransmission_delay, uint8_t retranmission_repeat);
static void NRF_SetDataPipe(uint8_t pipe_number);
static void NRF_EnableAutoACK(uint8_t pipe_number, uint8_t enable);
static void NRF_SetPayloadSize(uint8_t pipe_number, uint8_t size);
static void NRF_SetChannel(uint8_t channel);
static void NRF_SetAddressWidth(uint8_t address_width);
static void NRF_PowerUp(uint8_t power_up);

static void NRF_SetRXAddress(uint8_t pipe, uint8_t* address); // Remember to define RX address before TX
static void NRF_SetTXAddress(uint8_t* address);

static void NRF_RX_Mode(void);
static void NRF_TX_Mode(void);

static void NRF_FlushRX(void);
static void NRF_FlushTX(void);

static void NRF_TXPayload(uint8_t *data);
static void NRF_ReadRXPaylaod(uint8_t *data);

static uint8_t NRF_DataAvailable(void);

static void NRF_ClearRetransmissionFlag(void);

static void NRF_WriteDataToTXBuffer(uint8_t *buffer);

void NRF_Init(SPI_HandleTypeDef *hspi, char mode)
{
	NRF_spi = hspi;

	NRF_CE_LOW;
	NRF_SetOutputPower(NRF24_PA_PWR_0dBM);
	NRF_SetDataRate(NRF24_RF_DR_250KBPS);
	NRF_SetCRC(CRC_ENABLED, NRF24_CRC_WIDTH_1B);
	NRF_SetRetransmission(NRF24_RETRANSMISSION_DELAY, NRF24_RETRANMISSION_REPEAT);

	NRF_SetDataPipe(NRF24_PIPE_NUMBER);
	NRF_EnableAutoACK(NRF24_PIPE_NUMBER, NRF24_EN_AUTO_ACK);
	NRF_SetPayloadSize(NRF24_PIPE_NUMBER, NRF24_PAYLOAD_SIZE);

	NRF_SetChannel(NRF24_CHANNEL);
	NRF_SetAddressWidth(NRF24_ADD_WIDTH);

	NRF_SetRXAddress(NRF24_PIPE_NUMBER, (uint8_t*)"Nad");
	NRF_SetTXAddress((uint8_t*)"Odb");

	nrf_mode = mode;
	if(nrf_mode == 't')
	{
		NRF_TX_Mode();
		printf("Configured as transmitter \r\n");
	}
	else if (nrf_mode == 'r')
	{
		NRF_RX_Mode();
		printf("Configured as receiver \r\n");
	}
	else
		printf("No mode selected \r\n");
}

void NRF_process()
{
	if(NRF_Faults != NRF_NO_ERROR)
		NRF_State = NRF_IDLE;

	switch (NRF_State)
	{
	case NRF_POWER_DOWN:
		NRF_PowerUp(NRF24_POWER_ON);
		lastTick100us = Clock_GetTick();
		NRF_State = NRF_START_UP;
		break;

	case NRF_START_UP:
		if(Clock_GetTick() - lastTick100us >= NRF_START_UP_TIME)
			NRF_State = NRF_STANBY1;
		break;

	case NRF_STANBY1:
		lastTick100us = Clock_GetTick();
		if(nrf_mode == 't')
		{
			if(message_transmit == true)
			{
				if(message_length == NRF24_PAYLOAD_SIZE)
				{
					NRF_TXPayload(tx_buffer);
					NRF_CE_HIGH
					NRF_State = NRF_TX_SETTING;
				}
				else
				{
					NRF_Faults = NRF_DIFFRENT_MESSAGE_SIZE;
				}
			}
			else
			{
				NRF_CE_LOW
				NRF_State = NRF_STANBY1;
			}
		}
		else if(nrf_mode == 'r')
		{
			NRF_CE_HIGH
			NRF_State = NRF_RX_SETTING;
		}
		else
			NRF_Faults = NRF_NO_MODE;
		break;

	case NRF_TX_SETTING:
		if(Clock_GetTick() - lastTick100us >= NRF_SETTING_TIME)
		{
			NRF_State = NRF_TX_MODE;
			last_tick = HAL_GetTick();
		}
		break;

	case NRF_TX_MODE:;
		uint8_t fifo_status = NRF_ReadRegister(NRF24_FIFO_STATUS);
		uint8_t status = NRF_ReadStatusRegister();

		if(status & (1 << NRF24_MAX_RT))
			NRF_Faults = NRF_MAX_RETRANSMITS_FLAG;

		if(status & (1 << NRF24_TX_FULL))
			NRF_Faults = NRF_TX_FIFO_FULL;

		if((status & (1 << NRF24_TX_DS)) && (fifo_status & (1 << NRF24_TX_EMPTY)))
		{
			message_send_counter++;
			message_transmit = false; //already transmitted a packet
			NRF_CE_LOW
			NRF_State = NRF_STANBY1;
		}

		NRF_CalculateBitrate();
		break;

	case NRF_RX_SETTING:
		if(Clock_GetTick() - lastTick100us >= NRF_SETTING_TIME)
			NRF_State = NRF_RX_MODE;
		break;

	case NRF_RX_MODE:
		if(NRF_DataAvailable())
		{
			NRF_ReadRXPaylaod(rx_buffer);
			printf("Received message: %c \n", rx_buffer[0]);
		}
		break;

	case NRF_IDLE:
			switch(NRF_Faults)
			{
				case NRF_NO_MODE:
					printf("NRF_NO_MODE \r\n");
				break;

				case NRF_DIFFRENT_MESSAGE_SIZE:
					printf("NRF_DIFFRENT_MESSAGE_SIZE \r\n");
					break;

				case NRF_MAX_RETRANSMITS_FLAG:
					printf("NRF_MAX_RETRANSMITS_FLAG \r\n");
					NRF_ClearRetransmissionFlag();
					NRF_State = NRF_TX_MODE;
					NRF_Faults = NRF_NO_ERROR;
					break;

				case NRF_TX_FIFO_FULL:
					printf("NRF_TX_FIFO_FULL \r\n");
					NRF_FlushTX();
					NRF_State = NRF_TX_MODE;
					NRF_Faults = NRF_NO_ERROR;
					break;

				default:
				break;
			}
	}
}

void NRF_SendMessage(uint8_t* message, uint8_t message_len)
{
	NRF_WriteDataToTXBuffer(message);
	message_length = message_len;
	message_transmit = true;
}

static void NRF_CalculateBitrate(void)
{
	if(HAL_GetTick() - message_send_tick >= BITRATE_INTERVAL)
	{
		printf("Bitrate: %d messages per %d milliseconds \n", message_send_counter, BITRATE_INTERVAL);
		message_send_tick = HAL_GetTick();
		message_send_counter = 0;
	}
}

static void NRF_Read(uint8_t *data, uint8_t length)
{
	HAL_SPI_Receive(NRF_spi, data, length, 1000);
}

static void NRF_Write(uint8_t *data, uint8_t length)
{
	HAL_SPI_Transmit(NRF_spi, data, length, 1000);
}

static uint8_t NRF_ReadRegister(uint8_t reg)
{
	uint8_t reg_value;

	reg = NRF24_CMD_R_REGISTER | reg;
	NRF_CSN_LOW
	NRF_Write(&reg, 1);
	NRF_Read(&reg_value, 1);
	NRF_CSN_HIGH

	return reg_value;
}

static void NRF_ReadRegisters(uint8_t reg, uint8_t* data, uint8_t length)
{
	reg = NRF24_CMD_R_REGISTER | reg;
	NRF_CSN_LOW
	NRF_Write(&reg, 1);
	NRF_Read(data, length);
	NRF_CSN_HIGH
}

static void NRF_WriteRegister(uint8_t reg, uint8_t data)
{
	uint8_t buffer[2];
	buffer[0] = NRF24_CMD_W_REGISTER | reg;
	buffer[1] = data;

	NRF_CSN_LOW
	NRF_Write(buffer, 2);
	NRF_CSN_HIGH
}

static void NRF_WriteRegisters(uint8_t reg, uint8_t* data, uint8_t length)
{
	reg = NRF24_CMD_W_REGISTER | reg;

	NRF_CSN_LOW
	NRF_Write(&reg, 1);
	NRF_Write(data, length);
	NRF_CSN_HIGH
}

static void NRF_SendCommand(uint8_t command)
{
	NRF_CSN_LOW
	NRF_Write(&command, 1);
	NRF_CSN_HIGH
}

static uint8_t NRF_ReadConfigRegister(void)
{
	return NRF_ReadRegister(NRF24_CONFIG);
}

static void NRF_WriteConfigRegister(uint8_t config)
{
	NRF_WriteRegister(NRF24_CONFIG, config);
}

static uint8_t NRF_ReadStatusRegister(void)
{
	return (NRF_ReadRegister(NRF24_STATUS));
}

static void NRF_WriteStatusRegister(uint8_t status)
{
	NRF_WriteRegister(NRF24_STATUS, status);
}

static void NRF_SetOutputPower(uint8_t output_power)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_RF_SETUP);
	reg_value |= (output_power << 1);
	NRF_WriteRegister(NRF24_RF_SETUP, reg_value);
}

static void NRF_SetDataRate(uint8_t data_rate)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_RF_SETUP);

	if(data_rate == NRF24_RF_DR_250KBPS)
		reg_value |= (1 << NRF24_RF_DR_LOW);
	else if(data_rate == NRF24_RF_DR_2MBPS)
		reg_value |= (1 << NRF24_RF_DR_HIGH);
	else //NRF24_RF_DR_1MBPS
	{
		reg_value &= ~(1 << NRF24_RF_DR_LOW);
		reg_value &= ~(1 << NRF24_RF_DR_HIGH);
	}

	NRF_WriteRegister(NRF24_RF_SETUP, reg_value);
}

static void NRF_SetCRC(uint8_t crc_en, uint8_t crc_length)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_CONFIG);

	if(crc_en == NRF24_EN_CRC)
	{
		reg_value |= (1 << NRF24_EN_CRC);
		if(crc_length == NRF24_CRC_WIDTH_2B)
			reg_value |= (1 << NRF24_CRCO);
		else
			reg_value &= ~(1 << NRF24_CRCO);
	}
	else
	{
		reg_value &= ~(1 << NRF24_EN_CRC);
	}
	NRF_WriteRegister(NRF24_CONFIG, reg_value);
}

static void NRF_SetRetransmission(uint8_t retransmission_delay, uint8_t retranmission_repeat)
{
	uint8_t reg_value = 0;

	reg_value = (retransmission_delay << 4) ;
	reg_value |= retranmission_repeat;

	NRF_WriteRegister(NRF24_SETUP_RETR, reg_value);
}

static void NRF_SetDataPipe(uint8_t pipe_number)
{
	uint8_t reg_value = 0;
	reg_value |= (1 << pipe_number);
	NRF_WriteRegister(NRF24_EN_RXADDR, reg_value);
}

static void NRF_EnableAutoACK(uint8_t pipe_number, uint8_t enable)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_EN_AA);
	if(enable)
		reg_value |= (1 << pipe_number);
	else
		reg_value &= ~(1 << pipe_number);

	NRF_WriteRegister(NRF24_EN_AA, reg_value);
}

static void NRF_SetPayloadSize(uint8_t pipe_number, uint8_t size)
{
	NRF_WriteRegister((NRF24_RX_PW_P0 + pipe_number), size);
}

static void NRF_SetChannel(uint8_t channel)
{
	NRF_WriteRegister(NRF24_RF_CH, channel);
}

static void NRF_SetAddressWidth(uint8_t address_width)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_SETUP_AW);

	reg_value |= address_width;
	NRF_WriteRegister(NRF24_SETUP_AW, (reg_value - 2));
}

static void NRF_PowerUp(uint8_t power_up)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_CONFIG);
	reg_value |= (power_up << NRF24_PWR_UP);
	NRF_WriteRegister(NRF24_CONFIG, reg_value);
}

static void NRF_SetRXAddress(uint8_t pipe, uint8_t* address)
{
	if((pipe == 0) || (pipe == 1))
	{
		uint8_t address_rev[NRF24_ADD_WIDTH];
		for(uint8_t i = 0; i<NRF24_ADD_WIDTH; i++)
			address_rev[NRF24_ADD_WIDTH - 1 - i] = address[i];
		NRF_WriteRegisters(NRF24_RX_ADDR_P0 + pipe, address_rev, NRF24_ADD_WIDTH);
	}
	else
		NRF_WriteRegister(NRF24_RX_ADDR_P0 + pipe, address[NRF24_ADD_WIDTH-1]);
}

static void NRF_SetTXAddress(uint8_t* address)
{
	uint8_t address_rev[NRF24_ADD_WIDTH];

	NRF_ReadRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADD_WIDTH);

	for(uint8_t i = 0; i<NRF24_ADD_WIDTH; i++)
		addr_p0_backup[NRF24_ADD_WIDTH - 1 - i] = address_rev[i];

	for(uint8_t i = 0; i<NRF24_ADD_WIDTH; i++)
		address_rev[NRF24_ADD_WIDTH - 1 - i] = address[i];

	NRF_WriteRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADD_WIDTH);
	NRF_WriteRegisters(NRF24_TX_ADDR, address_rev, NRF24_ADD_WIDTH);
}

static void NRF_RX_Mode(void)
{
	uint8_t config = NRF_ReadConfigRegister();
	NRF_SetRXAddress(0, addr_p0_backup);
	config |= (1<<NRF24_PRIM_RX);

	NRF_WriteConfigRegister(config);

	NRF_WriteStatusRegister((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT));

	NRF_FlushRX();
	NRF_FlushTX();
}

static void NRF_TX_Mode(void)
{
	uint8_t config = NRF_ReadConfigRegister();
	config &= ~(1<<NRF24_PRIM_RX);
	NRF_WriteConfigRegister(config);

	NRF_WriteStatusRegister((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT));

	NRF_FlushRX();
	NRF_FlushTX();
}

static void NRF_FlushRX(void)
{
	NRF_SendCommand(NRF24_CMD_FLUSH_RX);
}

static void NRF_FlushTX(void)
{
	NRF_SendCommand(NRF24_CMD_FLUSH_TX);
}

static void NRF_TXPayload(uint8_t *data)
{
	NRF_WriteRegisters(NRF24_CMD_W_TX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
}

static void NRF_ReadRXPaylaod(uint8_t *data)
{
	NRF_ReadRegisters(NRF24_CMD_R_RX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
	NRF_WriteRegister(NRF24_STATUS, (1<NRF24_RX_DR));
	if(NRF_ReadStatusRegister() & (1<<NRF24_TX_DS))
		NRF_WriteRegister(NRF24_STATUS, (1<<NRF24_TX_DS));
}

static uint8_t NRF_DataAvailable(void)
{
	uint8_t status = NRF_ReadStatusRegister();
	if ((status & (1 << 6)))
	{
		status |= (1<<6);
		NRF_WriteStatusRegister(status);
		return 1;
	}
	return 0;
}

static void NRF_ClearRetransmissionFlag(void)
{
	uint8_t status = NRF_ReadStatusRegister();
	status |= (1 << NRF24_MAX_RT);
	NRF_WriteStatusRegister(status);
}

static void NRF_WriteDataToTXBuffer(uint8_t* buffer)
{
	for(uint8_t i = 0; i < NRF24_PAYLOAD_SIZE; i++)
		tx_buffer[i] = buffer[i];
}
