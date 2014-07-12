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
#include <Wire.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <L3G4200D.h>
#include <Servo.h> 
 
#define bool boolean

/**********************************************************************
RF serialization
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

union PacketSerializer{
  Packet data;
  uint8_t serialized[6];
};

/**********************************************************************
PID helper objects
***********************************************************************/

#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0

class PidObject{
  float _desired;      //< set point
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
public:
  PidObject(const float kp, const float ki, const float kd, const float intLimit):
    _error(0),
    _prevError(0),
    _integ(0),
    _deriv(0),
    _desired(0),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _iLimit(intLimit),
    _iLimitLow(-DEFAULT_PID_INTEGRATION_LIMIT)
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
  float Update(const float measured, const float dt, const bool updateError){

    if (updateError)
      _error = _desired - measured;

    _integ += _error * dt;
    if (_integ > _iLimit)
      _integ = _iLimit;
    else if (_integ < _iLimitLow)
      _integ = _iLimitLow;

    _deriv = (_error - _prevError) / dt;

    _outP = _kp * _error;
    _outI = _ki * _integ;
    _outD = _kd * _deriv;

    float output = _outP + _outI + _outD;

    _prevError = _error;

    return output;
  }
};

/**********************************************************************
Motor controller
***********************************************************************/

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700

// Управление моторами - включает калибровку ESC
class Motors{
  // Motors
  Servo _motor[4];
public:
  
  void Init(){
    _motor[0].attach(6);
    _motor[1].attach(5);
    _motor[2].attach(10);
    _motor[3].attach(9);
    calibrateESC();
  }

  // центр - 0 от -255 до 255
  void Update(uint16_t thr, int16_t yaw, int16_t roll, int16_t pitch){
    if (thr == 0){
      for(int i=0;i<4;++i)
        commandEsc(i, MIN_SIGNAL);
      return;
    }
    #define L(x,l) (x>l?l:(x<-l?-l:x));
    yaw = L(yaw, 255);
    roll = L(roll, 255);
    pitch = L(pitch, 255);
    #define MIXPWM(r,p,y) map(255 + thr + roll * r + pitch* p +  yaw * y, 0, 8*255, MIN_SIGNAL, MAX_SIGNAL)
    Serial.print(" PWM:");
    commandEsc(0, MIXPWM(-1,+1,-1));
    commandEsc(1, MIXPWM(-1,-1,+1));
    commandEsc(2, MIXPWM(+1,+1,+1));
    commandEsc(3, MIXPWM(+1,-1,-1));
  }
private:
  // Калибровка контроллеров скорости.
  void calibrateESC(){
    // Посылаем максимальный сигнал и минимальный - это описано в инструкции.
    for(int i=0; i<4; ++i)
      commandEsc(i, MAX_SIGNAL);
    delay(4000);
    for(int i=0; i<4; ++i)
      commandEsc(i, MIN_SIGNAL);
    delay(3000);    
  }

  inline void commandEsc(uint8_t id, uint16_t pwm){
    Serial.print(id, DEC);
    Serial.print(" ");
    Serial.print(pwm, DEC);
    //motor[id].writeMicroseconds(pwm);
  }
};

/**********************************************************************
nRF
***********************************************************************/
class RadioLink{
public:
  void Init(){
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
  }

  bool Update(){
    if(/*!Mirf.isSending() &&*/ Mirf.dataReady()){ // TODO: Attantion critical change.
    
      /*
       * Get load the packet into the buffer.
       */
       
      Mirf.getData(packet.serialized);
      
      /*
       * Wait untill sending has finished
       *
       * NB: isSending returns the chip to receving after returning true.
       */
       return true; // Data Readed
    }
    return false;
  }
  // RF message
  PacketSerializer packet; // TODO: Move to controller.
};

/**********************************************************************
Gyro stabilizer - most simplest one
***********************************************************************/

class Stabilizer
{
  L3G4200D _gyroscope;
  unsigned long _lastUpdateTime;
public:

  float X;
  float Y;
  float Z;

  Stabilizer():
    X(0.0f), Y(0.0f), Z(0.0f), _lastUpdateTime(0){}

  void Init(){
    _gyroscope.calibrate(100);
  }

  void Reset(){
    X = Y = Z = 0.0f;
  }

  void Update(){
    Vector norm = _gyroscope.readNormalize();
    unsigned long t = millis();
    float dt = (t - _lastUpdateTime) / 1000.0f;
    if (_lastUpdateTime == 0){
      _lastUpdateTime = t;
      return;
    }
    _lastUpdateTime = t;
    
    // Calculate Pitch, Roll and Yaw
    X = X + norm.XAxis * dt;
    Y = Y + norm.YAxis * dt;
    Z = Z + norm.ZAxis * dt;
  }
};

/**********************************************************************
Main Controller
***********************************************************************/

#define PID_ROLL_RATE_KP  5.0f
#define PID_ROLL_RATE_KI  0.1f
#define PID_ROLL_RATE_KD  0.1f
#define PID_ROLL_RATE_INTEGRATION_LIMIT    100.0f

#define PID_PITCH_RATE_KP  5.0f
#define PID_PITCH_RATE_KI  0.1f
#define PID_PITCH_RATE_KD  0.1f
#define PID_PITCH_RATE_INTEGRATION_LIMIT   100.0f

#define PID_YAW_RATE_KP  50.0f
#define PID_YAW_RATE_KI  25.0f
#define PID_YAW_RATE_KD  0.1f
#define PID_YAW_RATE_INTEGRATION_LIMIT     500.0f


#define RC_LIMITS_WAIT 3000 // Время ожидания калибровки
#define MIN_RC_DIFF 500 // Минимально допустимая разница.

#define MAX_ANGLE 0.5235f //(PI/6.0) - Максимальный угол отклонения от нуля.
#define SCALE_BACK 487 // 255*6/PI - Обратное масштабирование из радиан в [-255,255]

// Вспомогательный тип - пара значений мин макс для джойстика - нужно для определения диапазона джойстика.
class RcAxisLimits{
public:
  RcAxisLimits():Min(0),Max(0), Center(0), Scale(0){}

  void Update(uint16_t value){
    Min = min(value, Min);
    Max = max(value, Max);
    Center = (Min + Max) / 2;
    uint16_t div = Max - Min;
    if (div != 0)
      Scale = 2.0f*MAX_ANGLE / (div);
  }

  inline float Rescale(uint16_t value){
    return Scale * (float)((int16_t)value - (int16_t)Center);
  }

  uint16_t Min;
  uint16_t Max;
  uint16_t Center;
  float Scale; // Преобразование к углу по формуле: Rad = (RC - Center)*Scale
};


class Controller{
  PidObject _yaw, _pitch, _roll;
  Motors _motors;
  RadioLink _link;
  Stabilizer _stabilizer;
  RcAxisLimits _yawLimit, _pitchLimit, _rollLimit, _throttleLimit;
  long long _lastUpdateTime;
public:

  Controller():
    _roll(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, PID_ROLL_RATE_INTEGRATION_LIMIT),
    _pitch(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, PID_PITCH_RATE_INTEGRATION_LIMIT),
    _yaw(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, PID_YAW_RATE_INTEGRATION_LIMIT),
    _lastUpdateTime(0)
  {}

  void Init(){
    // Init motors
    _motors.Init();
    // Init nRF2400
    _link.Init();
    // Wait first packet
    while (_link.Update()) {}
    // Calibrate RC sticks limits.
    long long startMillis = millis();
    do{  
      _link.Update();
      Packet& refPacket = _link.packet.data;
      _throttleLimit.Update(refPacket.THR);
      _yawLimit.Update(refPacket.YAW);
      _rollLimit.Update(refPacket.ROL);
      _pitchLimit.Update(refPacket.PTC);
    } while(millis() - startMillis < RC_LIMITS_WAIT);
    // Ждем инициализации системы стабилизации...
    _stabilizer.Init();
    _lastUpdateTime = micros();
  }

  void Update(){
    _link.Update();
    Packet& refPacket = _link.packet.data;
    _stabilizer.Update();
    long long mic = micros();
    const float dt = (mic - _lastUpdateTime)/1000000.0f;
    const float yaw = _yawLimit.Rescale(refPacket.YAW);
    const float pitch = _pitchLimit.Rescale(refPacket.PTC);
    const float roll = _rollLimit.Rescale(refPacket.ROL);
    _yaw.SetDesired(yaw);
    _pitch.SetDesired(pitch);
    _roll.SetDesired(roll);
    const float pidYaw = _yaw.Update(-_stabilizer.X, dt, true);
    const float pidPitch = _pitch.Update(-_stabilizer.Z, dt, true);
    const float pidRoll = _roll.Update(-_stabilizer.Y, dt, true);
    const uint16_t yawMotor = scaleBack(pidYaw);
    const uint16_t pitchMotor = scaleBack(pidPitch);
    const uint16_t rollMotor = scaleBack(pidRoll);

    _motors.Update(refPacket.THR, yawMotor, rollMotor, pitchMotor);
  }

  // рескейл от радиан к -255 +255
  uint16_t scaleBack(float value){
    return SCALE_BACK * value;
  }

};

// Quadro controller
Controller TheController;

void setup(){
  Serial.begin(115200);
  
  TheController.Init();
  
  Serial.println("Listening...");
}

void loop(){
  TheController.Update();
}
