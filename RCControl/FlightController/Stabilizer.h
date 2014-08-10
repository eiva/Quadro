/**********************************************************************
Gyro stabilizer - most simplest one
***********************************************************************/
#include "OpenIMU.h"

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
    
    
    /*int16_t ax, ay, az;
    _accel.getAcceleration(&ax, &ay, &az);
    // ATTANTION: Works only for buggy ADX345 accelereometer with more than 4G offset.
    const float scale = 306.5f;
    const float fx = -(ax + 72.5f + 2.0f)   * 0.977671  / scale;
    const float fy = -(ay - 179.5f + 10.0f) * 0.977671  / scale;
    const float fz = -(az - 510.5f + 12.0f) * 1.08881f  / scale;
    const float fx2 = fx * fx;
    const float aPitch = atan2(fy, sqrt(fx2 + fz * fz));
    const float aRoll  = atan2(fz, sqrt(fx2 + fy * fy));*/
    const float aPitch = 0;
    const float aRoll  = 0;

	const Vector norm = _gyroscope.readNormalize();
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
