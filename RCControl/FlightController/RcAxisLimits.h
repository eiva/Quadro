
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

  inline String Info() const{
    return String(Min) + " " + String(Max);
  }

  uint16_t Min;
  uint16_t Max;
  uint16_t Center;
private:
  uint16_t _distance;
  uint16_t _last;
  float _scale; // Преобразование к углу по формуле: Rad = (RC - Center)*Scale
};
