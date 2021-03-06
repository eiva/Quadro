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



class Controller{
  PidObject _yaw, _pitch, _roll;
  Motors _motors;
  RadioLink _link;
  Stabilizer _stabilizer;
  RcAxisLimits _yawLimit, _pitchLimit, _rollLimit, _throttleLimit;
  LedInfo _info;
  long long _lastUpdateTime;
  BlackBox _blackBox;
public:

  Controller():
    _roll(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, PID_ROLL_RATE_INTEGRATION_LIMIT),
    _pitch(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, PID_PITCH_RATE_INTEGRATION_LIMIT),
    _yaw(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, PID_YAW_RATE_INTEGRATION_LIMIT),
    _lastUpdateTime(0),
    _motors(_info)
  {}

  void Init(){
    _blackBox.Init();
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

    _blackBox.Log(String("LT: " + _throttleLimit.Info() + "Y: " + _yawLimit.Info() + "R: " + _rollLimit.Info() + "P: " + _pitchLimit.Info() +"\n"));
    
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
    const float throttle = map(refPacket.THR, _throttleLimit.Min, _throttleLimit.Max, 0, 1024);
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
    _blackBox.Log(ftos(dt) + ", " + ftos(throttle) + ", " + ftos(yaw)+ ", " + ftos(pitch)+ ", " + ftos(roll) + ", " + ftos(stabYaw) + ", " + ftos(stabPitch) + ", " + ftos(stabRoll) + "\n");
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
    
    _motors.Update(throttle, motorYaw, motorRoll, motorPitch);
  }

  // рескейл от радиан к -255 +255
  int16_t scaleBack(float value){
    const float conv = SCALE_BACK * value;
    return max(-255.0f, min(255.0f, conv));
  }

};
