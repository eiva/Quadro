#pragma once
#include <math.h>

struct LogData
{
	uint32_t Timer; // Current timer time.
	uint16_t InputThrottle, InputYaw, InputPitch, InputRoll; // Input RC values.
	float GyroX, GyroY, GyroZ; // Gyro value.
	float AccelX, AccelY, AccelZ; // Accelerometer value.
	float MagX, MagY, MagZ; // Magnetometer value.
	float Alt; // Altitude.
	float Yaw, Roll, Pitch; // AHRS output.
	uint16_t Motor1, Motor2, Motor3, Motor4; // Motors output.
};

typedef struct _FIL FIL; // Forward declaration

class Logger
{
	bool _isDetected;
	FIL* _file;
public:
	Logger();
	void Log(const LogData& data);
};
