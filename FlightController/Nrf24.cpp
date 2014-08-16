#include <string.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_spi.h>
#include "Port.h"
#include "SpiInterface.h"
#include "Nrg24.h"

/* nRF24L0 commands */
#define nRF24_CMD_RREG             0x00  // R_REGISTER -> Read command and status registers
#define nRF24_CMD_WREG             0x20  // W_REGISTER -> Write command and status registers
#define nRF24_CMD_R_RX_PAYLOAD     0x61  // R_RX_PAYLOAD -> Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     0xA0  // W_TX_PAYLOAD -> Write TX payload
#define nRF24_CMD_FLUSH_TX         0xE1  // FLUSH_TX -> Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         0xE2  // FLUSH_RX -> Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      0xE3  // REUSE_TX_PL -> Reuse last transmitted payload
#define nRF24_CMD_NOP              0xFF  // No operation (to read status register)

/* nRF24L0 registers */
#define nRF24_REG_CONFIG           0x00  // Configuration register
#define nRF24_REG_EN_AA            0x01  // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        0x02  // Enable RX addresses
#define nRF24_REG_SETUP_AW         0x03  // Setup of address widths
#define nRF24_REG_SETUP_RETR       0x04  // Setup of automatic retranslation
#define nRF24_REG_RF_CH            0x05  // RF channel
#define nRF24_REG_RF_SETUP         0x06  // RF setup register
#define nRF24_REG_STATUS           0x07  // Status register
#define nRF24_REG_OBSERVE_TX       0x08  // Transmit observe register
#define nRF24_REG_CD               0x09  // Carrier detect
#define nRF24_REG_RX_ADDR_P0       0x0A  // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       0x0B  // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       0x0C  // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       0x0D  // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       0x0E  // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       0x0F  // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          0x10  // Transmit address
#define nRF24_REG_RX_PW_P0         0x11  // Number of bytes in RX payload id data pipe 0
#define nRF24_REG_RX_PW_P1         0x12  // Number of bytes in RX payload id data pipe 1
#define nRF24_REG_RX_PW_P2         0x13  // Number of bytes in RX payload id data pipe 2
#define nRF24_REG_RX_PW_P3         0x14  // Number of bytes in RX payload id data pipe 3
#define nRF24_REG_RX_PW_P4         0x15  // Number of bytes in RX payload id data pipe 4
#define nRF24_REG_RX_PW_P5         0x16  // Number of bytes in RX payload id data pipe 5
#define nRF24_REG_FIFO_STATUS      0x17  // FIFO status register
#define nRF24_REG_DYNPD            0x1C  // Enable dynamic payload length
#define nRF24_REG_FEATURE          0x1D  // Feature register

/* nRF24L0 bits */
#define nRF24_MASK_RX_DR           0x40  // Mask interrupt caused by RX_DR
#define nRF24_MASK_TX_DS           0x20  // Mask interrupt caused by TX_DS
#define nRF24_MASK_MAX_RT          0x10  // Mask interrupt caused by MAX_RT
#define nRF24_FIFO_RX_EMPTY        0x01  // RX FIFO empty flag
#define nRF24_FIFO_RX_FULL         0x02  // RX FIFO full flag

// Settings

#define RF_CHANNEL 0x01 // Channel #1
#define RF_TRANSMIT_MODE 0x0F


Nrf24::Nrf24(SpiInterface &spi, Port& csn, Port& ce):
	_spi(spi),
	_csn(csn),
	_ce(ce)
{
	CSN_H();
	CE_L();
}

void Nrf24::SetRxAddress(uint8_t rxAddr[nRF24_RX_ADDR_WIDTH]){
	memcpy(nRF24_RX_addr, rxAddr, nRF24_RX_ADDR_WIDTH);
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   0 - looks like an nRF24L01 is online
//   1 - received sequence differs from original
bool Nrf24::Check() {
	uint8_t txbuf[5] = { 0xA9,0xA9,0xA9,0xA9,0xA9 };
	uint8_t rxbuf[5];
	uint8_t i;

	WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,txbuf,5); // Write fake TX address
    ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Try to read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return false;

    return true;
}

// Put nRF24L01 in RX mode
void Nrf24::RXMode(uint8_t RX_PAYLOAD) {
	CE_L();

	WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P1, nRF24_RX_addr, nRF24_RX_ADDR_WIDTH); // Set static RX address

	RWReg(nRF24_CMD_WREG | nRF24_REG_EN_RXADDR,0x03);//0x01); // Enable data pipe 0

	//nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
	RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,RF_CHANNEL); // Set frequency channel 110 (2.510MHz)

	RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P0,RX_PAYLOAD); // Set RX payload length (10 bytes)
	RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P1,RX_PAYLOAD); // Set RX payload length (10 bytes)

	//nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP, RF_TRANSMIT_MODE);//0x06); // Setup: 1Mbps, 0dBm, LNA off
	RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG, 0x0B); // Config: CRC on (1 bytes), Power UP, RX/TX ctl = PRX

	CE_H();

	RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS, (1 << 5) | (1 << 4));

	CSN_L();
	_spi.ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
	CSN_H();
}

// Check if data is available for reading
// return:
//   false -> no data
//   true -> RX_DR is set or some bytes present in FIFO
bool Nrf24::IsDataReady() {
    uint8_t status;

    status = ReadReg(nRF24_REG_STATUS);
    if (status & nRF24_MASK_RX_DR) return 1;

    // Checking RX_DR isn't good enough, there's can be some data in FIFO
    status = ReadReg(nRF24_REG_FIFO_STATUS);

    return (status & nRF24_FIFO_RX_EMPTY) ? false : true;
}

bool Nrf24::RXPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD) {
	uint8_t status;

	status = ReadReg(nRF24_REG_STATUS); // Read status register
    if (status & nRF24_MASK_RX_DR) {
    	//if ((status & 0x0E) == 0) {
    		// pipe 0
    		ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD); // read received payload from RX FIFO buffer
    	//}
    	CSN_L();
		_spi.ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
		CSN_H();
		RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
	    //return nRF24_MASK_RX_DR;
	    return true;
    }

    // Some banana happens
    CSN_L();
    _spi.ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
    CSN_H();
	RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    return false;
}

// Clear all IRQ flags
void Nrf24::ClearIRQFlags() {
	uint8_t status;

    status = ReadReg(nRF24_REG_STATUS);
	RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
}


	// Write new value to register
	// input:
	//   reg - register number
	//   value - new value
	// output: nRF24L01 status
	uint8_t Nrf24::RWReg(uint8_t reg, uint8_t value) {
		uint8_t status;

		CSN_L();
		status = _spi.ReadWrite(reg); // Select register
		_spi.ReadWrite(value); // Write value to register
		CSN_H();

		return status;
	}

	// Read nRF24L01 register
	// input:
	//   reg - register number
	// output: register value
	uint8_t Nrf24::ReadReg(uint8_t reg) {
		uint8_t value;

		CSN_L();
		_spi.ReadWrite(reg);
		value = _spi.ReadWrite(0x00);
		CSN_H();

		return value;
	}

	// Get data from nRF24L01 into buffer
	// input:
	//   reg - register number
	//   pBuf - pointer to buffer
	//   count - bytes count
	// output: nRF24L01 status
	uint8_t Nrf24::ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
		uint8_t status,i;

		CSN_L();
		status = _spi.ReadWrite(reg);
		for (i = 0; i < count; i++) pBuf[i] = _spi.ReadWrite(0);
		CSN_H();

		return status;
	}

	// Send buffer to nRF24L01
	// input:
	//   reg - register number
	//   pBuf - pointer to buffer
	//   count - bytes count
	// output: nRF24L01 status
	uint8_t Nrf24::WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
		uint8_t status,i;

		CSN_L();
		status = _spi.ReadWrite(reg);
		for (i = 0; i < count; i++) _spi.ReadWrite(*pBuf++);
		CSN_H();

		return status;
	}

