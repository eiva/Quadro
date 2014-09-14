#pragma once

// Communication interface for SPI.
// http://easystm32.ru/interfaces/45-spi-interface-part-2
class SpiInterface{
	SPI_TypeDef* _spi;
public:
	SpiInterface(SPI_TypeDef* spi,
				 uint16_t prescaller = SPI_BaudRatePrescaler_8,
				 uint16_t cpol = SPI_CPOL_Low,
				 uint16_t cpha = SPI_CPHA_1Edge);

	// Send/Receive data via SPI
	// input:
	//  data - byte to send
	//  output: received byte from SPI
	uint8_t ReadWrite(uint8_t data);
};
