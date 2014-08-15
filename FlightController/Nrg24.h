#pragma once

class Nrf24{
	SpiInterface& _spi;
	Port _csn;
	Port _ce;
public:
	Nrf24(SpiInterface &spi, Port& csn, Port& ce):
		_spi(spi),
		_csn(csn),
		_ce(ce)
	{
		CSN_H();
		CE_L();
	}

	// Check if nRF24L01 present (send byte sequence, read it back and compare)
	// return:
	//   0 - looks like an nRF24L01 is online
	//   1 - received sequence differs from original
	uint8_t Check(void);
private:
	// Write new value to register
	// input:
	//   reg - register number
	//   value - new value
	// output: nRF24L01 status
	uint8_t RWReg(uint8_t reg, uint8_t value);

	// Read nRF24L01 register
	// input:
	//   reg - register number
	// output: register value
	uint8_t ReadReg(uint8_t reg);

	// Get data from nRF24L01 into buffer
	// input:
	//   reg - register number
	//   pBuf - pointer to buffer
	//   count - bytes count
	// output: nRF24L01 status
	uint8_t ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);

	// Send buffer to nRF24L01
	// input:
	//   reg - register number
	//   pBuf - pointer to buffer
	//   count - bytes count
	// output: nRF24L01 status
	uint8_t WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);

	// Chip Enable Activates RX or TX mode
	void inline CE_L(){_ce.Low();}
	void inline CE_H(){_ce.High();}
	// SPI Chip Select
	void inline CSN_L(){_csn.Low();}
	void inline CSN_H(){_csn.High();}
};
