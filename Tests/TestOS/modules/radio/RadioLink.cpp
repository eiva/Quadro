#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include <misc.h>
#include "Port.h"
#include "LedInfo.h"
#include "SpiInterface.h"
#include "Nrf24.h"
#include "RadioLink.h"




	RadioLink::RadioLink(Nrf24 &nrf, LedInfo &leds):
		_leds(leds),
		_nrf(nrf)
	{
		_leds.RGBY(false, false, false, true);
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
		_nrf.RXMode(sizeof(Packet), 10); // Channel 10.
		_leds.Off();
	}

	bool RadioLink::Update(){
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
