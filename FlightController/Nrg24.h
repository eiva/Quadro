#pragma once

/* Some constants */
#define nRF24_RX_ADDR_WIDTH        5    // nRF24 RX address width
#define nRF24_TX_ADDR_WIDTH        5    // nRF24 TX address width

class Nrf24{
	SpiInterface _spi;
	Port _csn;
	Port _ce;
	uint8_t nRF24_RX_addr[nRF24_RX_ADDR_WIDTH];
	//uint8_t nRF24_TX_addr[nRF24_TX_ADDR_WIDTH];
public:
	Nrf24(SpiInterface &spi, Port& csn, Port& ce);

	void SetRxAddress(uint8_t rxAddr[nRF24_RX_ADDR_WIDTH]);

	// Check if nRF24L01 present (send byte sequence, read it back and compare)
	// return:
	//   true - looks like an nRF24L01 is online
	//   false - received sequence differs from original
	bool Check();

	// Put nRF24L01 in RX mode
	void RXMode(uint8_t RX_PAYLOAD);

	// Check if data is available for reading
	// return:
	//   false -> no data
	//   true -> RX_DR is set or some bytes present in FIFO
	bool IsDataReady();

	// Read current packet from FIFO.
	// returns:
	//   true -> if data read.
	//   false -> if something goes wrong.
	bool RXPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD);

	// Clear all IRQ flags
	void ClearIRQFlags();

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
