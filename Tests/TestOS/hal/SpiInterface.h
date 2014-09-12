#pragma once

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"

// Communication interface for SPI.
// http://easystm32.ru/interfaces/45-spi-interface-part-2
class SpiInterface{
	SPI_TypeDef* _spi;
public:
	SpiInterface(SPI_TypeDef* spi,
				 uint16_t prescaller = SPI_BaudRatePrescaler_8,
				 uint16_t cpol = SPI_CPOL_Low,
				 uint16_t cpha = SPI_CPHA_1Edge):
		_spi(spi)
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_TypeDef* gpioPort;

		GPIO_StructInit(&GPIO_InitStruct);
		// Enable RCC clocking.
		if (_spi == SPI1)
		{
			gpioPort = GPIOA;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
		}
		else if (_spi == SPI2)
		{
			gpioPort = GPIOB;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
		}
		else
		{
			while(1);
		}

		// Setup SPI GPIO ports
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(gpioPort, &GPIO_InitStruct);

		// Init SPI port.
		// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
		SPI_InitTypeDef SPI;
		SPI_StructInit(&SPI);
		SPI.SPI_Mode = SPI_Mode_Master; // transmit in master mode, NSS pin has to be always high
		// Take care about prescaler. If CLK if too high it may be required to slow down SPI clock.
		SPI.SPI_BaudRatePrescaler = prescaller; // SPI frequency
		SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
		// Set SPI to Mode 0
		SPI.SPI_CPOL = cpol;
		SPI.SPI_CPHA = cpha;
		SPI.SPI_CRCPolynomial = 7;
		SPI.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
		SPI.SPI_FirstBit = SPI_FirstBit_MSB; // data is transmitted MSB first
		SPI.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
		SPI_Init(_spi, &SPI);

		SPI_Cmd(_spi,ENABLE);
	}

	// Send/Receive data via SPI
	// input:
	//  data - byte to send
	//  output: received byte from SPI
	uint8_t ReadWrite(uint8_t data){
		while (SPI_I2S_GetFlagStatus(_spi, SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
		SPI_I2S_SendData(_spi, data); // Send byte to SPI
		while (SPI_I2S_GetFlagStatus(_spi, SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
		return SPI_I2S_ReceiveData(_spi); // Read byte from SPI bus
	}
};
