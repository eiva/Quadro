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


//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels

#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 5  //set PPM signal output pin on the arduino

#define default_servo_value 1500  //set the default servo value
#define  SigMin  1000 // Min value for signal
#define SigMax  2000 // Max value for signal
//////////////////////////////////////////////////////////////////

 
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

union PacketSerializer
{
  Packet data;
  uint8_t serialized[6];
};

// Address of reciever.
byte addr[]={0xDB,0xDB,0xDB,0xDB,0xDB};

PacketSerializer packet;


void setup(){

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
  /*
  _thr.attach(3);
  _yaw.attach(5);
  _pitch.attach(6);
  _roll.attach(9);
  
  _thr.write(0);
  _yaw.write(0);
  _pitch.write(0);
  _roll.write(0);
  
  Serial.println("Beginning ... "); 
  */
  
    //initiallize default ppm values
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }
  ppm[0] = SigMin; // Throttle = 0;

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

const int THR = 2;
const int YAW = 3;
const int PTC = 1;
const int ROL = 0;
const int BT1 = 4;
const int BT2 = 5;
const int BT3 = 6;

void loop(){
  unsigned long time = millis();

  delay(10);

  while(!Mirf.dataReady()){
    //Serial.println("Waiting");
    if ( ( millis() - time ) > 1000 ) {
      //Serial.println("Timeout on response from server!");
      //_thr.write(_min);
      ppm[THR] = 0; // How to enable failover?
      return;
    }
  }
  
  Mirf.getData((byte *)packet.serialized);
  
  ppm[THR] = map(packet.data.THR,0,1024,SigMin,SigMax);
  ppm[YAW] = map(packet.data.YAW,0,1024,SigMin,SigMax+300);  
  ppm[PTC] = map(packet.data.PTC,0,1024,SigMin,SigMax);
  ppm[ROL] = map(packet.data.ROL,0,1024,SigMin,SigMax);
  ppm[BT1] = packet.data.BT1?SigMax:SigMin;
  ppm[BT2] = packet.data.BT2?SigMax:SigMin;
  ppm[BT3] = packet.data.BT3?SigMax:SigMin;
  
/*
  _thr.write(  map(packet.data.THR,0,1024,_min,_max));
  _yaw.write(  map(packet.data.YAW,0,1024,_min,_max));  
  _pitch.write(map(packet.data.PTC,0,1024,_min,_max));
  _roll.write( map(packet.data.ROL,0,1024,_min,_max));
  */
  //Serial.print(".");
} 
  
ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

  
