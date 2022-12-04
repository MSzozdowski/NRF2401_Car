/*
 * nRF24.c
 *
 *  Created on: Apr 26, 2020
 *      Author: Mateusz Salamon
 */

#include <NRF2401/nrf2401.h>
#include <NRF2401/nrf2401_defs.h>
#include "main.h"
#include "spi.h"

SPI_HandleTypeDef *NRF_spi;

static uint8_t addr_p0_backup[NRF24_ADD_WIDTH];

void NRF_Write(uint8_t *Data, uint8_t Length)
{
	HAL_SPI_Transmit(NRF_spi, Data, Length, 1000);
}

void NRF_Read(uint8_t *Data, uint8_t Length)
{
	HAL_SPI_Receive(NRF_spi, Data, Length, 1000);
}

uint8_t NRF_ReadRegister(uint8_t reg)
{
	uint8_t reg_value;

	reg = NRF24_CMD_R_REGISTER | reg;
	NRF_CSN_LOW;
	NRF_Write(&reg, 1);
	NRF_Read(&reg_value, 1);
	NRF_CSN_HIGH;

	return reg_value;
}

void NRF_ReadRegisters(uint8_t reg, uint8_t* data, uint8_t length)
{
	reg = NRF24_CMD_R_REGISTER | reg;
	NRF_CSN_LOW;
	NRF_Write(&reg, 1);
	NRF_Read(data, length);
	NRF_CSN_HIGH;
}

void NRF_WriteRegister(uint8_t reg, uint8_t data)
{
	uint8_t buffer[2];
	buffer[0] = NRF24_CMD_W_REGISTER | reg;
	buffer[1] = data;

	NRF_CSN_LOW;
	NRF_Write(buffer, 2);
	NRF_CSN_HIGH;
}

void NRF_WriteRegisters(uint8_t reg, uint8_t* data, uint8_t length)
{
	reg = NRF24_CMD_W_REGISTER | reg;

	NRF_CSN_LOW;
	NRF_Write(&reg, 1);
	NRF_Write(data, length);
	NRF_CSN_HIGH;
}

void NRF_SendCommand(uint8_t command)
{
	NRF_CSN_LOW;
	NRF_Write(&command, 1);
	NRF_CSN_HIGH;
}

void NRF_TX_Mode(void)
{
	uint8_t config = NRF_ReadConfigRegister();
	config &= ~(1<<NRF24_PRIM_RX);
	NRF_WriteConfig(config);

	NRF_WriteStatus((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT));

	NRF_FlushRX();
	NRF_FlushTX();
}

void NRF_RX_Mode(void)
{
	uint8_t config = NRF_ReadConfigRegister();
	NRF_SetRXAddress(0, addr_p0_backup);
	config |= (1<<NRF24_PWR_UP); //wywalenia
	config |= (1<<NRF24_PRIM_RX);

	NRF_WriteConfig(config);

	NRF_WriteStatus((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT));

	NRF_FlushRX();
	NRF_FlushTX();

	NRF_CE_HIGH; //wywalenia
	HAL_Delay(1); //wywalenia
}

uint8_t NRF_ReadConfigRegister()
{
	return NRF_ReadRegister(NRF24_CONFIG);
}


void NRF_WriteConfig(uint8_t config)
{
	NRF_WriteRegister(NRF24_CONFIG, config);
}

void NRF_SetOutputPower(uint8_t output_power)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_RF_SETUP);
	reg_value |= (output_power << 1);
	NRF_WriteRegister(NRF24_RF_SETUP, reg_value);
}

void NRF_SetDataRate(uint8_t data_rate)
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

uint8_t NRF_ReadStatus(void)
{
	return (NRF_ReadRegister(NRF24_STATUS));
}

void NRF_WriteStatus(uint8_t status)
{
	NRF_WriteRegister(NRF24_STATUS, status);
}

void NRF_FlushRX(void)
{
	NRF_SendCommand(NRF24_CMD_FLUSH_RX);
}

void NRF_FlushTX(void)
{
	NRF_SendCommand(NRF24_CMD_FLUSH_TX);
}

void NRF_SetCRC(uint8_t crc_en, uint8_t crc_length)
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

void NRF_SetRetransmission(uint8_t retransmission_delay, uint8_t retranmission_repeat)
{
	uint8_t reg_value = 0;

	reg_value = (retransmission_delay << 4) ;
	reg_value |= retranmission_repeat;

	NRF_WriteRegister(NRF24_SETUP_RETR, reg_value);
}

void NRF_SetChannel(uint8_t channel)
{
	NRF_WriteRegister(NRF24_RF_CH, channel);
}

void NRF_SetPayloadSize(uint8_t pipe_number, uint8_t size)
{
	NRF_WriteRegister((NRF24_RX_PW_P0 + pipe_number), size);
}

void NRF_SetDataPipe(uint8_t pipe_number)
{
	uint8_t reg_value = 0;
	reg_value |= (1 << pipe_number);
	NRF_WriteRegister(NRF24_EN_RXADDR, reg_value);
}

void NRF_EnableAutoACK(uint8_t pipe_number, uint8_t enable)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_EN_AA);
	if(enable)
		reg_value |= (1 << pipe_number);
	else
		reg_value &= ~(1 << pipe_number);

	NRF_WriteRegister(NRF24_EN_AA, reg_value);
}

void NRF_SetAddressWidth(uint8_t address_width)
{
	uint8_t reg_value = NRF_ReadRegister(NRF24_SETUP_AW);

	reg_value |= address_width;
	NRF_WriteRegister(NRF24_SETUP_AW, (reg_value - 2));
}

void NRF_SetRXAddress(uint8_t pipe, uint8_t* address)
{
	if((pipe == 0) || (pipe == 1))
	{
		uint8_t i;
		uint8_t address_rev[NRF24_ADD_WIDTH];
		for(i = 0; i<NRF24_ADD_WIDTH; i++)
			address_rev[NRF24_ADD_WIDTH - 1 - i] = address[i];
		NRF_WriteRegisters(NRF24_RX_ADDR_P0 + pipe, address_rev, NRF24_ADD_WIDTH);
	}
	else
		NRF_WriteRegister(NRF24_RX_ADDR_P0 + pipe, address[NRF24_ADD_WIDTH-1]);
}

void NRF_SetTXAddress(uint8_t* address)
{
	uint8_t i;
	uint8_t address_rev[NRF24_ADD_WIDTH];

	NRF_ReadRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADD_WIDTH);

	for(i = 0; i<NRF24_ADD_WIDTH; i++)
		addr_p0_backup[NRF24_ADD_WIDTH - 1 - i] = address_rev[i];

	for(i = 0; i<NRF24_ADD_WIDTH; i++)
		address_rev[NRF24_ADD_WIDTH - 1 - i] = address[i];

	NRF_WriteRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADD_WIDTH);
	NRF_WriteRegisters(NRF24_TX_ADDR, address_rev, NRF24_ADD_WIDTH);
}

void NRF_TXPayload(uint8_t *data, uint8_t size)
{
	NRF_WriteRegisters(NRF24_CMD_W_TX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
}

void NRF_ReadRXPaylaod(uint8_t *data)
{
	NRF_ReadRegisters(NRF24_CMD_R_RX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
	NRF_WriteRegister(NRF24_STATUS, (1<NRF24_RX_DR));
	if(NRF_ReadStatus() & (1<<NRF24_TX_DS))
		NRF_WriteRegister(NRF24_STATUS, (1<<NRF24_TX_DS));
}

uint8_t NRF_DataAvailable(void)
{
	uint8_t status = NRF_ReadStatus();

	// RX FIFO Interrupt
	if ((status & (1 << 6)))
	{
		//nrf24_rx_flag = 1;
		status |= (1<<6); // Interrupt flag clear
		NRF_WriteStatus(status);
		return 1;
	}
	return 0;
}

void NRF_Init(SPI_HandleTypeDef *hspi)
{
	NRF_spi = hspi;

	NRF_CE_LOW;

	HAL_Delay(5); // Wait for radio power up

	NRF_SetOutputPower(NRF24_PA_PWR_0dBM);
	NRF_SetDataRate(NRF24_RF_DR_250KBPS);
	NRF_SetCRC(CRC_ENABLED, NRF24_CRC_WIDTH_1B);
	NRF_SetRetransmission(NRF24_RETRANSMISSION_DELAY, NRF24_RETRANMISSION_REPEAT);
	NRF_SetChannel(NRF24_CHANNEL);
	NRF_SetPayloadSize(NRF24_PIPE_NUMBER, NRF24_PAYLOAD_SIZE);
	NRF_SetDataPipe(NRF24_PIPE_NUMBER);
	NRF_EnableAutoACK(NRF24_PIPE_NUMBER, NRF24_EN_AUTO_ACK);

	NRF_SetAddressWidth(NRF24_ADD_WIDTH);

	HAL_Delay(20);
}
