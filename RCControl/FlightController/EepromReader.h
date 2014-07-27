
#pragma pack(push,1)

#define EEPROM_DATA_VERSION 1
struct EepromData{
  uint16_t ThrMin:10;
  uint16_t ThrMax:10;
  uint16_t YawMin:10;
  uint16_t YawMax:10;
  uint16_t PitchMin:10;
  uint16_t PitchMax:10;
  uint16_t RollMin:10;
  uint16_t RollMax:10;
};

#pragma pack(pop)

class EepromReader{

  union EepromSerializer{
    EepromData data;
    uint8_t serialized[sizeof(EepromData)];
  };

  EepromSerializer _buffer;
public:
  EepromReader(): Data(_buffer.data){}
  EepromData& Data;
  bool Load(){
    if (EEPROM.read(0) != EEPROM_DATA_VERSION) return false;
    const int start = 1;
    for (int i = 0; i < sizeof(EepromData); ++i){
      _buffer.serialized[i] = EEPROM.read(start + i);
    }
    return true;
  }
  void Reset(){
    EEPROM.write(0, 255);
  }
  void Store(){
    const int start = 1;
    EEPROM.write(0, EEPROM_DATA_VERSION);
    for (int i = 0; i < sizeof(EepromData); ++i){
      EEPROM.write(start +i, _buffer.serialized[i]);
    }
  }
};

