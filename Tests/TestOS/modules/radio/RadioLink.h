#pragma once

struct RadioLinkData
{
	uint16_t Throttle, Yaw, Pitch, Roll;
};

// Controls rf data link
class RadioLink{
	Nrf24 *_nrf;
	LedInfo *_leds;
public:
	RadioLink(Nrf24 *nrf, LedInfo *leds);
	bool Update(RadioLinkData& data);
};
