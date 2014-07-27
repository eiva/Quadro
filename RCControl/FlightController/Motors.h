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
      commandEsc(i, MIN_SIGNAL);
    delay(3000);    
  }

  inline void commandEsc(uint8_t id, uint16_t pwm){
    _motor[id].write(pwm);
  }
};
