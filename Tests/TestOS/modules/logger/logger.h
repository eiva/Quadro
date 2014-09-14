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

// Forward declarations
typedef struct _FIL FIL;
class Button;
class LedInfo;

class Logger
{
	bool _isDetected;
	bool _isMounted;
	FIL* _file;
	LedInfo *_ledInfo;
	Button *_button; // If button pressed - closes files, unmount
public:
	Logger(LedInfo *info, Button *button);
	void Log(const LogData& data);
private:
	void mount();
};
