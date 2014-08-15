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


// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   0 - looks like an nRF24L01 is online
//   1 - received sequence differs from original
uint8_t Nrf24::Check(void) {
	uint8_t txbuf[5] = { 0xA9,0xA9,0xA9,0xA9,0xA9 };
	uint8_t rxbuf[5];
	uint8_t i;

	WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,txbuf,5); // Write fake TX address
    ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Try to read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return 1;

    return 0;
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
		CSN_L();

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

