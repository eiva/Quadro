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

