#pragma once

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

// Controls rf data link
class RadioLink{
	union PacketSerializer
	{
	  Packet data;
	  uint8_t serialized[6];
	};

	Nrf24 &_nrf;
	LedInfo &_leds;
	PacketSerializer _serializer;
public:
	RadioLink(Nrf24 &nrf, LedInfo &leds);
	bool Update();

	uint16_t Throttle, Yaw, Pitch, Roll;
};
