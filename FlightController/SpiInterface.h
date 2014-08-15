#pragma once

#include <stm32f10x_spi.h>

class SpiInterface{
	SPI_TypeDef* _spi;
public:
	SpiInterface(SPI_TypeDef* spi, GPIO_TypeDef* gpioPort, uint16_t miso, uint16_t mosi, uint16_t sck):
		_spi(spi)
	{
		GPIO_InitTypeDef port;
		GPIO_StructInit(&port);
		port.GPIO_Speed = GPIO_Speed_50MHz;

		// Setup SPI GPIO ports
		port.GPIO_Pin = sck | mosi;
		port.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(gpioPort,&port);

		port.GPIO_Pin = miso;
		port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(gpioPort,&port);

		// Init SPI port.
		SPI_InitTypeDef SPI;
		SPI_StructInit(&SPI);
		SPI.SPI_Mode = SPI_Mode_Master;
		SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

		// Set SPI to Mode 0
		SPI.SPI_CPOL = SPI_CPOL_Low;
		SPI.SPI_CPHA = SPI_CPHA_1Edge;
		SPI.SPI_CRCPolynomial = 7;
		SPI.SPI_DataSize = SPI_DataSize_8b;
		SPI.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI.SPI_NSS = SPI_NSS_Soft;
		SPI_Init(_spi,&SPI);

		// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
		SPI_NSSInternalSoftwareConfig(_spi,SPI_NSSInternalSoft_Set);

		SPI_Cmd(_spi,ENABLE);
	}

	// Send/Receive data via SPI
	// input:
	//  data - byte to send
	//  output: received byte from nRF24L01
	uint8_t ReadWrite(uint8_t data){
		while (SPI_I2S_GetFlagStatus(_spi, SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
		SPI_I2S_SendData(_spi,data); // Send byte to SPI
		while (SPI_I2S_GetFlagStatus(_spi, SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
		return SPI_I2S_ReceiveData(_spi); // Read byte from SPI bus
	}
};
