//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#pragma once

class MadwickAHRS
{
	float beta;				// algorithm gain

public:
	MadwickAHRS();
	void MadgwickAHRSupdate(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

private:
	void MadgwickAHRSupdateIMU(float dT, float gx, float gy, float gz, float ax, float ay, float az);
};

//=====================================================================================================
// End of file
//=====================================================================================================
