/*
 * nrf2401_defs.h
 *
 *  Created on: Nov 12, 2022
 *      Author: MSzozdowski
 */

#ifndef INC_NRF2401_NRF2401_DEFS_H_
#define INC_NRF2401_NRF2401_DEFS_H_

//
//Config
//
#define NRF24_CHANNEL 10				//0-127
#define NRF24_PIPE_NUMBER 0 			//0-5
#define NRF24_EN_AUTO_ACK 1				//0-OFF, 1-ON
#define NRF24_ADD_WIDTH 3				//00-3bytes 10-4bytes 11-5bytes
#define NRF24_RETRANSMISSION_DELAY 4 	//0-15 each value is 250uS
#define NRF24_RETRANMISSION_REPEAT 7 	//0-15 times
#define NRF24_PAYLOAD_SIZE 2			//0-32

#define CRC_ENABLED 1
#define CRC_DISABLED 0

#define NRF24_POWER_ON	1
#define NRF24_POWER_OFF	0

//
//Delays
//
#define NRF_START_UP_TIME 2000
#define NRF_SETTING_TIME 200

// Registers
//
#define NRF24_CONFIG		0x00
#define NRF24_EN_AA			0x01
#define NRF24_EN_RXADDR		0x02
#define NRF24_SETUP_AW		0x03
#define NRF24_SETUP_RETR	0x04
#define NRF24_RF_CH			0x05
#define NRF24_RF_SETUP		0x06
#define NRF24_STATUS		0x07
#define NRF24_OBSERVE_TX	0x08
#define NRF24_CD			0x09
#define NRF24_RX_ADDR_P0	0x0A
#define NRF24_RX_ADDR_P1	0x0B
#define NRF24_RX_ADDR_P2	0x0C
#define NRF24_RX_ADDR_P3	0x0D
#define NRF24_RX_ADDR_P4	0x0E
#define NRF24_RX_ADDR_P5	0x0F
#define NRF24_TX_ADDR		0x10
#define NRF24_RX_PW_P0		0x11
#define NRF24_RX_PW_P1		0x12
#define NRF24_RX_PW_P2		0x13
#define NRF24_RX_PW_P3		0x14
#define NRF24_RX_PW_P4		0x15
#define NRF24_RX_PW_P5		0x16
#define NRF24_FIFO_STATUS	0x17
#define NRF24_DYNPD			0x1C
#define NRF24_FEATURE		0x1D

//
// Commands
//
#define NRF24_CMD_R_REGISTER			0x00
#define NRF24_CMD_W_REGISTER			0x20
#define NRF24_CMD_R_RX_PAYLOAD			0x61
#define NRF24_CMD_W_TX_PAYLOAD			0xA0
#define NRF24_CMD_FLUSH_TX				0xE1
#define NRF24_CMD_FLUSH_RX				0xE2
#define NRF24_CMD_REUSE_TX_PL			0xE3
#define NRF24_CMD_ACTIVATE				0x50
#define NRF24_CMD_R_RX_PL_WID			0x60
#define NRF24_CMD_W_ACK_PAYLOAD			0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK	0xB0
#define NRF24_CMD_NOP					0xFF

//
// Bit Mnemonics
//
#define NRF24_MASK_RX_DR  6
#define NRF24_MASK_TX_DS  5
#define NRF24_MASK_MAX_RT 4
#define NRF24_EN_CRC      3
#define NRF24_CRCO        2
#define NRF24_PWR_UP      1
#define NRF24_PRIM_RX     0
#define NRF24_ENAA_P5     5
#define NRF24_ENAA_P4     4
#define NRF24_ENAA_P3     3
#define NRF24_ENAA_P2     2
#define NRF24_ENAA_P1     1
#define NRF24_ENAA_P0     0
#define NRF24_ERX_P5      5
#define NRF24_ERX_P4      4
#define NRF24_ERX_P3      3
#define NRF24_ERX_P2      2
#define NRF24_ERX_P1      1
#define NRF24_ERX_P0      0
#define NRF24_AW          0
#define NRF24_ARD         4
#define NRF24_ARC         0
#define NRF24_PLL_LOCK    4
#define NRF24_RF_DR_HIGH  3
#define NRF24_RF_DR_LOW	  5
#define NRF24_RF_PWR      1
#define NRF24_LNA_HCURR   0
#define NRF24_RX_DR       6
#define NRF24_TX_DS       5
#define NRF24_MAX_RT      4
#define NRF24_RX_P_NO     1
#define NRF24_TX_FULL     0
#define NRF24_PLOS_CNT    4
#define NRF24_ARC_CNT     0
#define NRF24_TX_REUSE    6
#define NRF24_FIFO_FULL   5
#define NRF24_TX_EMPTY    4
#define NRF24_RX_FULL     1
#define NRF24_RX_EMPTY    0
#define NRF24_RPD         0x09


#define NRF24_CRC_WIDTH_1B 0
#define NRF24_CRC_WIDTH_2B 1

#define NRF24_RF_DR_250KBPS 2
#define NRF24_RF_DR_1MBPS 0
#define NRF24_RF_DR_2MBPS 1

#define NRF24_PA_PWR_M18dBM 0
#define NRF24_PA_PWR_M12dBM 1
#define NRF24_PA_PWR_M6dBM 2
#define NRF24_PA_PWR_0dBM 3

#endif /* INC_NRF2401_NRF2401_DEFS_H_ */
