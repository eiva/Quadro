/**
 * nRF transmitter board. 
 * v 0.1
 * Eugene Ivanchenko, 2014
 *
 * Pins:
 * 1) nRF2400
 *    MISO -> 12
 *    MOSI -> 11
 *    SCK  -> 13
 *    CE   -> 10
 *    CSN  -> 8
 * 2) Joysticks
 *    THR  -> A4
 *    YAW  -> A5
 *    PTC  -> A3
 *    ROL  -> A2
 * 3) Buttons
 *    B1   -> 0
 *    B2   -> 1
 *    B3   -> 2
 */

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>


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

union PacketSerializer
{
  Packet data;
  uint8_t serialized[6];
};

// Address of reciever.
byte addr[]={0xDB,0xDB,0xDB,0xDB,0xDB};

// Joystick pins
const int THRpin = A4;
const int YAWpin = A5;
const int PTCpin = A3;
const int ROLpin = A2;

// Buttons pins
const int B1pin = 0;
const int B2pin = 1;
const int B3pin = 2;

// RF message
PacketSerializer packet;

void setup(){

  pinMode(B1pin, INPUT);
  pinMode(B2pin, INPUT);
  pinMode(B3pin, INPUT);
  
  Mirf.csnPin = 8;
  Mirf.cePin = 10;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  //Configure reciving address.
  Mirf.setRADDR(addr);

  Mirf.payload = 6; // Sizeof(Packet)

  Mirf.channel = 10; // any
 
  Mirf.config();
  Mirf.setTADDR(addr); // NB: In example it was before each send.
}

void loop(){
  delay(10); // For testing reasons
  while(Mirf.isSending()){} // Wait previous transmit

  // Read Joystick
  packet.data.THR = analogRead(THRpin);
  packet.data.YAW = analogRead(YAWpin);
  packet.data.PTC = analogRead(PTCpin);
  packet.data.ROL = analogRead(ROLpin);
  // Read buttons
  packet.data.BT1 = digitalRead(B1pin);
  packet.data.BT2 = digitalRead(B2pin);
  packet.data.BT3 = digitalRead(B3pin);
  // Send packet
  Mirf.send((byte *)packet.serialized);
} 
  
  
  
