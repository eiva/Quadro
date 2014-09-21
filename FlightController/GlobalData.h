#pragma once
class GlobalData
{
public:
	GlobalData()
	: BootMilliseconds(0)
	, AttQ0(0), AttQ1(0), AttQ2(0), AttQ3(0)
	{}
public:

	uint32_t BootMilliseconds; ///< Milliseconds from start.

	float AX, AY, AZ; ///< Accelerometer values.
	float GX, GY, GZ; ///< Gyro values.
	float MX, MY, MZ; ///< Magnetometer values.

	uint16_t RT, RY, RP, RR; ///< Raw radio link data (4 channel).

	float AttQ0, AttQ1, AttQ2, AttQ3; ///< Attitude quaternion values from AHRS
};

extern GlobalData TheGlobalData;
