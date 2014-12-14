/**
 * A Mirf example to test the latency between two Ardunio.
 *
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 *
 * Configurable:
 * CE -> 8
 * CSN -> 7
 *
 * Note: To see best case latency comment out all Serial.println
 * statements not displaying the result and load 
 * 'ping_server_interupt' on the server.
 */

#include <Servo.h> 
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

Servo _thr;
Servo _yaw;
Servo _pitch;
Servo _roll;

void setup(){
  Serial.begin(9600);
  /*
   * Setup pins / SPI.
   */
   
  /* To change CE / CSN Pins:
   * 
   * Mirf.csnPin = 9;
   * Mirf.cePin = 7;
   */
  /*
  Mirf.cePin = 7;
  Mirf.csnPin = 8;
  */
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  
  /*
   * Configure reciving address.
   */
   
  Mirf.setRADDR(addr);
  
  /*
   * Set the payload length to sizeof(unsigned long) the
   * return type of millis().
   *
   * NB: payload on client and server must be the same.
   */
   
  Mirf.payload = 6;
  
  /*
   * Write channel and payload config then power up reciver.
   */
   
  /*
   * To change channel:
   * 
   * Mirf.channel = 10;
   *
   * NB: Make sure channel is legal in your area.
   */
  Mirf.channel = 10; 
  Mirf.config();
  
  _thr.attach(3);
  _yaw.attach(5);
  _pitch.attach(6);
  _roll.attach(9);
  
  _thr.write(0);
  _yaw.write(0);
  _pitch.write(0);
  _roll.write(0);
  
  Serial.println("Beginning ... "); 
}

PacketSerializer packet;

void loop(){
  unsigned long time = millis();
  const int _min = 40;
  const int _max = 140;
  

  delay(10);
  while(!Mirf.dataReady()){
    //Serial.println("Waiting");
    if ( ( millis() - time ) > 1000 ) {
      Serial.println("Timeout on response from server!");
      _thr.write(_min);
      return;
    }
  }
  
  Mirf.getData((byte *)packet.serialized);
  

  _thr.write(  map(packet.data.THR,0,1024,_min,_max));
  _yaw.write(  map(packet.data.YAW,0,1024,_min,_max));  
  _pitch.write(map(packet.data.PTC,0,1024,_min,_max));
  _roll.write( map(packet.data.ROL,0,1024,_min,_max));
  //Serial.print(".");
} 
  
  
  
