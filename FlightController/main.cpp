#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_spi.h>
#include <misc.h>
#include "Utils.h"
#include "Port.h"
#include "LedInfo.h"
#include "Motors.h"
#include "ADCPort.h"
/*
 * Notepad
 * SystemCoreClock = 72000000
 */

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


int main(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	SysTick_Config(SystemCoreClock/1000);


	ADCPort adc(GPIOB, GPIO_Pin_0, ADC_Channel_8);
	Motors motor;

	LedInfo leds;
	while(true){
		Delay(50);

		uint16_t value = adc.Read();
		uint16_t motorVal = map(value, 0, 4095, 10, 1800);
		motor.SetRatio(motorVal, motorVal, 0, 1800);
		if (value <= 1024){
			leds.RGBW(true, false, false, false);
		}else if (value <=2*1024){
			leds.RGBW(false, true, false, false);
		}else if (value <=3*1024){
			leds.RGBW(false, false, true, false);
		}else{
			leds.RGBW(false, false, false, true);
		}
	}
/*
	int d = 1;
	int step = 5;
	int max = 1800;
	int min = 0;
	int r = min;
	leds.W(true);
	while(true){

		m.SetRatio(r, r, max, min);
		Delay(10);

		r += d;
		if (r >= max - 1){
			d = -step;
			leds.RGBW(true, false, false, false);
		}
		if (r <= min + 1){
			d = step;
			leds.RGBW(false, true, false, false);
		}
	}*/
}


