/**********************************************************************
RF serialization and EEPROM data
***********************************************************************/

#pragma pack(push,1)

struct Packet{
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

/**********************************************************************
nRF
***********************************************************************/
class RadioLink{
	union PacketSerializer{
		Packet data;
		uint8_t serialized[6];
	};
	PacketSerializer _buffer;
public:
  RadioLink():Data(_buffer.data){}

  // Init nRF MCU
  void Init(){
    // Set the SPI Driver.
    Mirf.spi = &MirfHardwareSpi;
    
    // Setup pins / SPI.
    Mirf.init();
    
    // Configure reciving address.
     
    byte addr[]={0xDB,0xDB,0xDB,0xDB,0xDB};
    Mirf.setRADDR(addr);
    
    
    // Set the payload length to sizeof(unsigned long) the
    Mirf.payload = 6; // size of (PacketSerializer)
    Mirf.channel = 10; // any
    // Write channel and payload config then power up reciver.
    Mirf.config();
  }

  // Read data from nRF: if no data returns false
  bool Update(){
    if(Mirf.dataReady()){
        // Get load the packet into the buffer.
        Mirf.getData(_buffer.serialized);
	    return true; // Data Readed
    }
    return false;
  }
  // RF message
  Packet& Data; // TODO: Move to controller.
};
