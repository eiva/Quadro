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
#include <I2Cdev.h>
#include <EEPROM.h>
#include <ADXL345.h>
 
#define bool boolean

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


/**********************************************************************
PID helper objects
***********************************************************************/

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
    _error(0), _prevError(0), _integ(0), _deriv(0), _desired(0), _kp(kp),
    _ki(ki), _kd(kd), _iLimit(intLimit), _iLimitLow(-intLimit)
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
Led information display
***********************************************************************/

class LedInfo{
 uint8_t _r, _g, _b, _w;
public:
  LedInfo():_r(3),_g(2),_b(4), _w(A3){}
  void Init(){
    pinMode(_r, OUTPUT);
    pinMode(_g, OUTPUT);
    pinMode(_b, OUTPUT);
    pinMode(_w, OUTPUT);
  }
  void R(bool on){
    digitalWrite(_r, on?HIGH:LOW);
  }
  void G(bool on){
    digitalWrite(_g, on?HIGH:LOW);
  }
  void B(bool on){
    digitalWrite(_b, on?HIGH:LOW);
  }
  void W(bool on){
    digitalWrite(_w, on?HIGH:LOW);
  }
  void RGBW(bool r, bool g, bool b, bool w){
    R(r); G(g); B(b); W(w);
  }
};

/**********************************************************************
Motor controller
***********************************************************************/

#define MAX_SIGNAL 1700 // out of 1800 due to protection form full throttle.
#define MIN_SIGNAL 880
#define OFF_SIGNAL 750

// Управление моторами - включает калибровку ESC
class Motors{
  // Motors
  Servo _motor[4];
  LedInfo& _info;
public:
  Motors(LedInfo& info):_info(info){}

  void Init(){
    _motor[0].attach(10);
    _motor[1].attach(9);
    _motor[2].attach(6);
    _motor[3].attach(5);
    _info.R(true);
    calibrateESC();
    _info.R(false);
  }

  // центр - 0 от -255 до 255
  void Update(uint16_t thr, int16_t yaw, int16_t roll, int16_t pitch){
    if (thr == 0){
      _info.B(true);
      _info.W(false);
      for(int i=0;i<4;++i)
        commandEsc(i, OFF_SIGNAL);
      return;
    }else{
      _info.B(false);
      _info.W(true);
    }
    #define L(x,l) (x>l?l:(x<-l?-l:x));
    yaw = L(yaw, 255);
    roll = L(roll, 255);
    pitch = L(pitch, 255);
    #define MIXPWM(r,p,y) map(255 + thr + roll * r + pitch* p +  yaw * y, 0, 8*255, MIN_SIGNAL, MAX_SIGNAL)
    const uint16_t m0 = MIXPWM(-1,+1,-1);
    const uint16_t m1 = MIXPWM(-1,-1,+1);
    const uint16_t m2 = MIXPWM(+1,+1,+1);
    const uint16_t m3 = MIXPWM(+1,-1,-1);
    commandEsc(0, m0);
    commandEsc(1, m1);
    commandEsc(2, m2);
    commandEsc(3, m3);
    /*Serial.print(" THR: ");
   	Serial.print(thr);
    Serial.print(" [PWM: ");
   	Serial.print(m0);
   	Serial.print(" ");
   	Serial.print(m1);
   	Serial.print(" ");
   	Serial.print(m2);
   	Serial.print(" ");
   	Serial.print(m3);
   	Serial.print(" ");
   	Serial.println("]");
   	delay(500);*/
  }
private:
  // Калибровка контроллеров скорости.
  void calibrateESC(){
    // Посылаем максимальный сигнал и минимальный - это описано в инструкции.
    for(int i=0; i<4; ++i)
      commandEsc(i, MAX_SIGNAL);
    delay(3000);
    for(int i=0; i<4; ++i)
      commandEsc(i, OFF_SIGNAL);
    delay(3000);    
  }

  inline void commandEsc(uint8_t id, uint16_t pwm){
    _motor[id].write(pwm);
  }
};

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

/**********************************************************************
Gyro stabilizer - most simplest one
***********************************************************************/

// TODO: Not used yet Kalman filter for axis.
class Kalman {
public:
    Kalman() {
        /* We will set the variables like so, these can also be tuned by the user */
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;
        Reset();
    };

    // Reset all filter values (not parameters) to it initials.
    void Reset(){
    	angle = 0; // Reset the angle
        bias = 0; // Reset bias

        P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 0;
    }

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float Update(float newAngle, float newRate, float dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P[0][0] + R_measure;
        /* Step 5 */
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;
        /* Step 6 */
        angle += K[0] * y;
        bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle;
    };
    void SetAngle(float newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    float GetRate() const { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void SetQangle(float newQ_angle) { Q_angle = newQ_angle; };
    void SetQbias(float newQ_bias) { Q_bias = newQ_bias; };
    void SetRmeasure(float newR_measure) { R_measure = newR_measure; };

    float GetQangle() const { return Q_angle; };
    float GetQbias() const { return Q_bias; };
    float GetRmeasure() const { return R_measure; };

private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    float K[2]; // Kalman gain - This is a 2x1 vector
    float y; // Angle difference
    float S; // Estimate error
};

// Complimentary filter realization.
class Complimentary{
	float _a, _b; // Alpha, Beta parameters.
	float _estimated; // Current estimated value.
public:
	// Init complimentary filter using Alpha, Beta parameters.
	Complimentary(float a, float b): _a(a), _b(b), _estimated(0.0f){}

	// Update estimation using current IMU values for gyro rate and accelereometer values.
	float Update(float gyro, float accel, float dt){
		_estimated = _a * (_estimated + gyro * dt) + _b * accel;
		return _estimated;
	}

	// Reset current estimation to zero.
	void Reset() { _estimated = 0.0f; }
};

#define COMP_APLHA 1.0f//0.7f // Alpha parameter for complimentary filter for Gyro
#define COMP_BETA 0.0f//0.3f // Beta parameter for complimentary filter for Acc
#define GRAD2RAD(grad) 0.01745329251994329576923690768489f * (grad) // Convert graduses to radians

// Реализация стабилизации движения - вычисления позиции устройства в пространстве.
class Stabilizer
{
  L3G4200D _gyroscope; // Gyroscope HAL
  ADXL345 _accel; // Accelereometer HAL
  unsigned long _lastUpdateTime; // Used for calculate dT
  Complimentary _pitchFilter; // Pitch estimation filter
  Complimentary _rollFilter; // Yaw estimation filter
public:

  float Yaw; // Current estimated values for Yaw
  float Pitch; // Current estimated values for Pitch
  float Roll; // Current estimated values for Roll

  Stabilizer():
    Yaw(0.0f), Pitch(0.0f), Roll(0.0f), _lastUpdateTime(0),
    _pitchFilter(COMP_APLHA, COMP_BETA), _rollFilter(COMP_APLHA, COMP_BETA){}

  void Init(){
    while(!_gyroscope.begin(L3G4200D_SCALE_250DPS, L3G4200D_DATARATE_800HZ_110)){
      delay(500);
    }
    delay(100);
    _gyroscope.calibrate(100);
    _gyroscope.setThreshold(0);
    _accel.setRange(ADXL345_RANGE_8G);
    _accel.setFullResolution(1);
    // ATTANTION: Due to my accel have huge estimation offset - need to be recalibrated
    _accel.setOffsetY(63);
    _accel.setOffsetZ(-128);
    delay(100);
  }

  void Reset(){
    Yaw = Pitch = Roll = 0.0f;
    //_yawFilter.Reset();
    _pitchFilter.Reset();
    _rollFilter.Reset();
  }

  void Update(){
    const Vector norm = _gyroscope.readNormalize();
    
    int16_t ax, ay, az;
    _accel.getAcceleration(&ax, &ay, &az);
    // ATTANTION: Works only for buggy ADX345 accelereometer with more than 4G offset.
    const float scale = 306.5f;
    const float fx = -(ax + 72.5f + 2.0f)   * 0.977671  / scale;
    const float fy = -(ay - 179.5f + 10.0f) * 0.977671  / scale;
    const float fz = -(az - 510.5f + 12.0f) * 1.08881f  / scale;
    const float fx2 = fx * fx;
    const float aPitch = atan2(fy, sqrt(fx2 + fz * fz));
    const float aRoll  = atan2(fz, sqrt(fx2 + fy * fy));

    const float gYaw   = - GRAD2RAD(norm.XAxis);
    const float gPitch =   GRAD2RAD(norm.ZAxis);
    const float gRoll  = - GRAD2RAD(norm.YAxis);

    const unsigned long t = micros();
    const float dt = (t - _lastUpdateTime) / 1000000.0f;

    if (_lastUpdateTime == 0){
       _lastUpdateTime = t;
       return;
    }
    _lastUpdateTime = t;
    
    // Calculate Pitch, Roll and Yaw
    Yaw = Yaw  + gYaw * dt; // TODO: Correct using magnetometer.
    Pitch = _pitchFilter.Update(gPitch, aPitch, dt);
    Roll = _rollFilter.Update(gRoll, aRoll, dt);
  }
};

/**********************************************************************
Main Controller
***********************************************************************/

#define PID_ROLL_RATE_KP  0.8f
#define PID_ROLL_RATE_KI  0.01f
#define PID_ROLL_RATE_KD  0.01f
#define PID_ROLL_RATE_INTEGRATION_LIMIT    0.5f

#define PID_PITCH_RATE_KP  0.8f
#define PID_PITCH_RATE_KI  0.01f
#define PID_PITCH_RATE_KD  0.01f
#define PID_PITCH_RATE_INTEGRATION_LIMIT   0.5f

#define PID_YAW_RATE_KP  0.4f
#define PID_YAW_RATE_KI  0.01f
#define PID_YAW_RATE_KD  0.01f
#define PID_YAW_RATE_INTEGRATION_LIMIT     0.5f


#define RC_LIMITS_WAIT 1000 // Время ожидания калибровки
#define MIN_RC_DIFF 500 // Минимально допустимая разница.

#define MAX_ANGLE 0.52359877559829887307710723054658f //(PI/6.0) - Максимальный угол отклонения от нуля.
#define SCALE_BACK  487.01412586119972745278431591989f// 255*6/PI - Обратное масштабирование из радиан в [-255,255]

// Вспомогательный тип - пара значений мин макс для джойстика - нужно для определения диапазона джойстика.
class RcAxisLimits{
public:
  RcAxisLimits():Min(0),Max(0),Center(0),_scale(0),_distance(0),_last(0){}
  
  // true - если изменились показания или макс-мин не удовлетворяют проверке.
  bool Update(const uint16_t value){
    Min = min(value, Min);
    Max = max(value, Max);
    const bool changed = abs(value - _last) > 10; // дребезг некоторый, по этому 10
    _last = value;
    Center = (Min + Max) / 2;
    _distance = Max - Min;
    if (_distance != 0)
      _scale = 2.0f*MAX_ANGLE / (_distance);
    return changed || _distance < MIN_RC_DIFF;
  }

  inline float Rescale(const uint16_t value) const{
    return _scale * (float)((int16_t)value - (int16_t)Center);
  }

  uint16_t Min;
  uint16_t Max;
  uint16_t Center;
private:
  uint16_t _distance;
  uint16_t _last;
  float _scale; // Преобразование к углу по формуле: Rad = (RC - Center)*Scale
};


class Controller{
  PidObject _yaw, _pitch, _roll;
  Motors _motors;
  RadioLink _link;
  Stabilizer _stabilizer;
  RcAxisLimits _yawLimit, _pitchLimit, _rollLimit, _throttleLimit;
  LedInfo _info;
  long long _lastUpdateTime;
public:

  Controller():
    _roll(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, PID_ROLL_RATE_INTEGRATION_LIMIT),
    _pitch(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, PID_PITCH_RATE_INTEGRATION_LIMIT),
    _yaw(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, PID_YAW_RATE_INTEGRATION_LIMIT),
    _lastUpdateTime(0),
    _motors(_info)
  {}

  void Init(){
    _info.Init();
    // Init motors
    _motors.Init();
    _info.RGBW(true, true, true, true);
    // Init nRF2400
    _link.Init();
    // Wait first packet
    while (!_link.Update()) {
      delay(250);
    }
    _info.RGBW(true, true, false, true);
    // Calibrate RC sticks limits.
    long long startMillis = millis();
    Packet& refPacket = _link.Data;
    do{  
      _link.Update();
      bool changed = _throttleLimit.Update(refPacket.THR);
      changed |= _yawLimit.Update(refPacket.YAW);
      changed |= _rollLimit.Update(refPacket.ROL);
      changed |= _pitchLimit.Update(refPacket.PTC);
      if (changed){
        startMillis = millis();
      }
    } while(millis() - startMillis < RC_LIMITS_WAIT);
    _info.RGBW(true, false, false, false);
    do{
      _link.Update();
    }while (refPacket.THR > _throttleLimit.Min);
    _info.RGBW(false, true, true, false);
    // Ждем инициализации системы стабилизации...
    _stabilizer.Init();
    _info.RGBW(false, false, false, false);
    _lastUpdateTime = micros();
  }

  void Update(){
    //const unsigned long t1 = micros(); // Profile
    _link.Update();
    Packet& refPacket = _link.Data;
    if (refPacket.THR == 0 || refPacket.BT3 != 0)
    {
      _stabilizer.Reset();
      _yaw.Reset();
      _pitch.Reset();
      _roll.Reset();
      _motors.Update(0, 0, 0, 0);
      return;
    }
    _stabilizer.Update();
    const unsigned long mic = micros();
    const float dt = (mic - _lastUpdateTime)/1000000.0f;
    const float yaw = _yawLimit.Rescale(refPacket.YAW);
    const float pitch = _pitchLimit.Rescale(refPacket.PTC);
    const float roll = _rollLimit.Rescale(refPacket.ROL);
    _yaw.SetDesired(yaw);
    _pitch.SetDesired(pitch);
    _roll.SetDesired(roll);
    const float stabYaw = _stabilizer.Yaw;
    const float stabRoll = _stabilizer.Roll;
    const float stabPitch = _stabilizer.Pitch;
    const float pidYaw = _yaw.Update(stabYaw, dt, true);
    const float pidPitch = _pitch.Update(stabPitch, dt, true);
    const float pidRoll = _roll.Update(stabRoll, dt, true);
    const int16_t motorYaw = scaleBack(pidYaw);
    const int16_t motorPitch = scaleBack(pidPitch);
    const int16_t motorRoll = scaleBack(pidRoll);
    
    /*const unsigned long spant = micros() - t1;
    Serial.print("Input ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" Stab ");
    Serial.print(stabYaw);
    Serial.print(" ");
    Serial.print(stabPitch);
    Serial.print(" ");
    Serial.print(stabRoll);
    Serial.print(" PID ");
    Serial.print(pidYaw);
    Serial.print(" ");
    Serial.print(pidPitch);
    Serial.print(" ");
    Serial.print(pidRoll);
    Serial.print(" Motor ");
    Serial.print(motorYaw);
    Serial.print(" ");
    Serial.print(motorPitch);
    Serial.print(" ");
    Serial.print(motorRoll);
    Serial.print(" dT ");
    Serial.print(spant);
    Serial.print(";");*/
    //delay(500);
    
    _motors.Update(refPacket.THR, motorYaw, motorRoll, motorPitch);
  }

  // рескейл от радиан к -255 +255
  int16_t scaleBack(float value){
    const float conv = SCALE_BACK * value;
    return max(-255.0f, min(255.0f, conv));
  }

};

// Quadro controller
Controller TheController;
//Stabilizer TheStab;

void setup(){
  Serial.begin(9600);
  
  TheController.Init();
  //TheStab.Init();
}

void loop(){
    TheController.Update();
	/*TheStab.Update();
	const float stabYaw = TheStab.Yaw;
    const float stabPitch = TheStab.Pitch;
    const float stabRoll = TheStab.Roll;
    Serial.print("YPR ");
    Serial.print(stabYaw);
    Serial.print(" ");
    Serial.print(stabPitch);
    Serial.print(" ");
    Serial.println(stabRoll);*/
}
