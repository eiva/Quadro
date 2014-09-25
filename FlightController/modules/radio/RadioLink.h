#pragma once

class LedInfo;
class Nrf24;

// Controls rf data link
class RadioLink{
	Nrf24 *_nrf;
	LedInfo *_leds;
public:
	RadioLink(Nrf24 *nrf, LedInfo *leds);
	bool Update();
	uint16_t Throttle, Yaw, Roll, Pitch; // RAW Values.
};
