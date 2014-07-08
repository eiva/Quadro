/**
 * An Mirf example which copies back the data it recives.
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
 */
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Servo.h> 
 
#define bool boolean

/**********************************************************************
RF serialization
***********************************************************************/

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

union PacketSerializer{
  Packet data;
  uint8_t serialized[6];
};

/**********************************************************************
PID helper objects
***********************************************************************/

#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0

class PidObject{
  float _desired;     //< set point
  float _error;        //< error
  float _prevError;    //< previous error
  float _integ;        //< integral
  float _deriv;        //< derivative
  float _kp;           //< proportional gain
  float _ki;           //< integral gain
  float _kd;           //< derivative gain
  float _outP;         //< proportional output (debugging)
  float _outI;         //< integral output (debugging)
  float _outD;         //< derivative output (debugging)
  float _iLimit;       //< integral limit
  float _iLimitLow;    //< integral limit
  float _dt;           //< delta-time dt
public:
  PidObject(const float kp, const float ki, const float kd, const float dt, const float intLimit):
    _error(0),
    _prevError(0),
    _integ(0),
    _deriv(0),
    _desired(0),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _iLimit(intLimit),
    _iLimitLow(-DEFAULT_PID_INTEGRATION_LIMIT),
    _dt(dt)
  { }

  /**
  * Reset the PID error values
  */
  void Reset(){
    _error     = 0;
    _prevError = 0;
    _integ     = 0;
    _deriv     = 0;
  }

 /**
  * Set a new set point for the PID to track.
  *
  * @param[in] desired The new desired value
  */
  void SetDesired(float desired){
    _desired = desired;
  }

  /**
   * Update the PID parameters.
   *
   * @param[in] pid         A pointer to the pid object.
   * @param[in] measured    The measured value
   * @param[in] updateError Set to TRUE if error should be calculated.
   *                        Set to False if pidSetError() has been used.
   * @return PID algorithm output
   */
  float pidUpdate(const float measured, const bool updateError){
    float output;

    if (updateError){
        _error = _desired - measured;
    }

    _integ += _error * _dt;
    if (_integ > _iLimit){
        _integ = _iLimit;
    } else if (_integ < _iLimitLow){
        _integ = _iLimitLow;
    }

    _deriv = (_error - _prevError) / _dt;

    _outP = _kp * _error;
    _outI = _ki * _integ;
    _outD = _kd * _deriv;

    output = _outP + _outI + _outD;

    _prevError = _error;

    return output;
  }

};

/**********************************************************************
Motor controller
***********************************************************************/

class RcAxisLimits{
  uint16_t _min;
  uint16_t _max;
  uint16_t _center;
public:
  RcAxisLimits():
    _min(0), _max(0),_center(0){}

// TODO: Надо сделать по времени: если 1 секунду позиция не меняется - то ок, идем дальше.
  bool Update(uint16_t value){
    uint16_t old = _max;
    _min = min(value, _min);
    _max = max(value, _max);
    if (old == 0)
      return false;
    if (_max > old)
      return false;
    _center = (_max + _min) / 2;
    return true;
  }

  uint16_t GetCenter() const {return _center;}
};


#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
class Motors{
  // Motors
  Servo _motor[4];

  RcAxisLimits _yaw, _pitch, _roll, _throttle;
public:
  Motors(){
    _motor[0].attach(6);
    _motor[1].attach(5);
    _motor[2].attach(10);
    _motor[3].attach(9);
  }

  void CalibrateESC(){
    for(int i=0;i<4;++i)
      commandEsc(i, MAX_SIGNAL);
    delay(4000);
    for(int i=0;i<4;++i)
      commandEsc(i, MIN_SIGNAL);
    delay(3000);    
  }

  void CalibrateRC(){

  }


  void SetMotors(uint16_t thr, uint16_t yaw, uint16_t roll, uint16_t pitch){
    #define MIXPWM(r,p,y) map(max(thr + (roll-512)* r + (pitch-512)* p +  (yaw-512) * y, 0), 0, 1024+3*512, MIN_SIGNAL, MAX_SIGNAL)
    Serial.print(" PWM:");
    commandEsc(0, MIXPWM(-1,+1,-1));
    commandEsc(1, MIXPWM(-1,-1,+1));
    commandEsc(2, MIXPWM(+1,+1,+1));
    commandEsc(3, MIXPWM(+1,-1,-1));
  }
private:
  inline void commandEsc(uint8_t id, uint16_t pwm){
    Serial.print(id, DEC);
    Serial.print(" ");
    Serial.print(pwm, DEC);
    //motor[id].writeMicroseconds(MAX_SIGNAL);
  }
};

/**********************************************************************
Main Controller
***********************************************************************/

#define PID_ROLL_RATE_KP  70.0
#define PID_ROLL_RATE_KI  0.0
#define PID_ROLL_RATE_KD  0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    100.0

#define PID_PITCH_RATE_KP  70.0
#define PID_PITCH_RATE_KI  0.0
#define PID_PITCH_RATE_KD  0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   100.0

#define PID_YAW_RATE_KP  50.0
#define PID_YAW_RATE_KI  25.0
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     500.0

#define IMU_UPDATE_DT 0.05

class Controller{
  PidObject _yaw;
  PidObject _pitch;
  PidObject _roll;
  Motors _motors;
public:

  Controller():
    _roll(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT, PID_ROLL_RATE_INTEGRATION_LIMIT),
    _pitch(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT, PID_PITCH_RATE_INTEGRATION_LIMIT),
    _yaw(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT, PID_YAW_RATE_INTEGRATION_LIMIT)
  {}
};


// Quadro controller
Controller TheController;

// RF message
PacketSerializer packet;


// INIT
boolean isArmed = false;

void setup(){
  Serial.begin(9600);
  
  /*
   * Set the SPI Driver.
   */

  Mirf.spi = &MirfHardwareSpi;
  
  /*
   * Setup pins / SPI.
   */
   
  Mirf.init();
  
  /*
   * Configure reciving address.
   */
   
  byte addr[]={0xDB,0xDB,0xDB,0xDB,0xDB};
  Mirf.setRADDR(addr);
  
  /*
   * Set the payload length to sizeof(unsigned long) the
   * return type of millis().
   *
   * NB: payload on client and server must be the same.
   */
   
  Mirf.payload = 6;
  Mirf.channel = 10; // any
  /*
   * Write channel and payload config then power up reciver.
   */
   
  Mirf.config();
  
  Serial.println("Listening...");
}





void reseiveRC(){
    if(!Mirf.isSending() && Mirf.dataReady()){
    
    /*
     * Get load the packet into the buffer.
     */
     
    Mirf.getData(packet.serialized);
    
    /*
     * Wait untill sending has finished
     *
     * NB: isSending returns the chip to receving after returning true.
     */
    }
}

void loop(){
  if (!isArmed){
//    arm();
    isArmed = true;
  }
  reseiveRC();
    Serial.print("Data:");
    Serial.print(packet.data.THR, DEC);
    Serial.print(" ");
    Serial.print(packet.data.YAW, DEC);
    Serial.print(" ");
    Serial.print(packet.data.PTC, DEC);
    Serial.print(" ");
    Serial.print(packet.data.ROL, DEC);
    Serial.print(" ");
    Serial.print(packet.data.BT1, DEC);
    Serial.print(" ");
    Serial.print(packet.data.BT2, DEC);
    Serial.print(" ");
    Serial.print(packet.data.BT3, DEC);

    

}
