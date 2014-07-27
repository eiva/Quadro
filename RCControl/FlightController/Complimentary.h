// Your perfect code here


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

