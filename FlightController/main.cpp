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
#include "SpiInterface.h"
#include "Nrg24.h"
/*
 * Notepad:
 * SystemCoreClock = 72000000
 */
#pragma pack(push,1)

struct Packet
{
  uint16_t THR:10;
  uint16_t YAW:10;
  uint16_t PTC:10;
  uint16_t ROL:10;
  uint8_t BT1:1;
  uint8_t BT2:1;
  uint8_t BT3:1;
  uint8_t REST:5;
};

#pragma pack(pop)

class RadioChannel{
	union PacketSerializer
	{
	  Packet data;
	  uint8_t serialized[6];
	};

	Port _csn;
	Port _ce;
	Nrf24 _nrf;
	LedInfo _leds;
	PacketSerializer _serializer;
public:
	RadioChannel(SpiInterface& spi, LedInfo& leds):
		_csn(GPIOA, GPIO_Pin_4),
		_ce(GPIOA, GPIO_Pin_8),
		_leds(leds),
		_nrf(spi, _csn, _ce)
	{
		_leds.RGBW(false, false, false, true);
		if( sizeof(Packet) != 6){
			// Failed!
			while(true);
		}

		uint8_t rxAddr[nRF24_RX_ADDR_WIDTH]={0xDB,0xDB,0xDB,0xDB,0xDB};
		_nrf.SetRxAddress(rxAddr);

		bool check = _nrf.Check();
		if (!check){
			_leds.R(true);
		}
		_nrf.RXMode(sizeof(Packet));
		_leds.Off();
	}

	bool Update(){
		if (!_nrf.IsDataReady()) return false;
		for (uint8_t i = 0; i < sizeof(Packet); ++i) _serializer.serialized[i] = 0x00;

		bool status = _nrf.RXPacket(_serializer.serialized, sizeof(Packet));
		//if (!status) return false;
		Throttle =  _serializer.data.THR;
		Yaw =  _serializer.data.YAW;
		Pitch =  _serializer.data.PTC;
		Roll =  _serializer.data.ROL;
		_nrf.ClearIRQFlags();
		return true;
	}

	uint16_t Throttle, Yaw, Pitch, Roll;
};

int main(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	SysTick_Config(SystemCoreClock/1000);


	LedInfo leds;

	SpiInterface spi(SPI1, GPIOA, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_5);
	RadioChannel chennel(spi, leds);
	leds.G(true);





	while(true){
		if (chennel.Update())
		{
			leds.B(true);
		}
	}
	/*ADCPort adc(GPIOB, GPIO_Pin_0, ADC_Channel_8);
	Motors motor;


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
	}*/
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


