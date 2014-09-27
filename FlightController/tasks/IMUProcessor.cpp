#include "stm32f4xx_conf.h"

#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "GlobalData.h"
#include "Port.h"
#include "LedInfo.h"
#include "SpiInterface.h"
#include "mpu9250.h"
#include "Helpers.h"
#include "Helpers.h"
#include <math.h>
#include "MadwickAHRS.h"
#include "IMUProcessor.h"

extern LedInfo* TheLedInfo;

void vTaskIMUProcessor (void *pvParameters)
{
	IMUData data;
	MadwickAHRS ahrs;
	Mpu9250 *mpu = (Mpu9250*)pvParameters;
	float lastTick = xTaskGetTickCount();
	uint8_t ReadBuf[21];
	while(1)
    {
		TheLedInfo->B(true);
		const float currentTick = xTaskGetTickCount();

		mpu->Read(ReadBuf);

		const float aRes = 4.0f/32768.0f;
		const float gRes = (radians(500.0f))/32768.0f;
		const float mRes = 0.15f;

		// https://mbed.org/users/kylongmu/code/MPU9250_SPI/file/084e8ba240c1/MPU9250.cpp

		TheGlobalData.BootMilliseconds = currentTick;

		TheGlobalData.AX = aRes * (float)Byte16ToInt16(ReadBuf[0],  ReadBuf[1]);  // Acc.X
		TheGlobalData.AY = aRes * (float)Byte16ToInt16(ReadBuf[2],  ReadBuf[3]);  // Acc.Y
		TheGlobalData.AZ = -aRes * (float)Byte16ToInt16(ReadBuf[4],  ReadBuf[5]);  // Acc.Z

		TheGlobalData.Temperature =(((float)Byte16ToInt16(ReadBuf[6],  ReadBuf[7])-21.0f)/333.87f)+21.0f;

		TheGlobalData.GX = -gRes * (float)Byte16ToInt16(ReadBuf[8],  ReadBuf[9]);  // Gyr.X
		TheGlobalData.GY = -gRes * (float)Byte16ToInt16(ReadBuf[10], ReadBuf[11]); // Gyr.Y
		TheGlobalData.GZ = gRes * (float)Byte16ToInt16(ReadBuf[12], ReadBuf[13]); // Gyr.Z

		// TODO: Need to be calibrated.
		TheGlobalData.MX = mRes * (float)Byte16ToInt16(ReadBuf[14],  ReadBuf[15]);  // Mag.X
		TheGlobalData.MY = mRes * (float)Byte16ToInt16(ReadBuf[16],  ReadBuf[17]);  // Mag.Y
		TheGlobalData.MZ = mRes * (float)Byte16ToInt16(ReadBuf[18],  ReadBuf[19]);  // Mag.Z

		// dT calculation
		const float dT = (currentTick - lastTick) / 1000.0f; // Seconds
		lastTick = currentTick;

		// AHRS update
		ahrs.MadgwickAHRSupdate(dT,
				TheGlobalData.GX, TheGlobalData.GY, TheGlobalData.GZ,
				TheGlobalData.AX, TheGlobalData.AY, TheGlobalData.AZ,
				//TheGlobalData.MX, TheGlobalData.MY, TheGlobalData.MZ);
				0, 0, 0);

		TheGlobalData.AttQ0 = ahrs.q0;
		TheGlobalData.AttQ1 = ahrs.q1;
		TheGlobalData.AttQ2 = ahrs.q2;
		TheGlobalData.AttQ3 = ahrs.q3;

		// Euler calculation...

		const float q23 = ahrs.q2 * ahrs.q3;
		const float q01 = ahrs.q0 * ahrs.q1;
		const float q13 = ahrs.q1 * ahrs.q3;
		const float q02 = ahrs.q0 * ahrs.q2;
		const float q12 = ahrs.q1 * ahrs.q2;
		const float q03 = ahrs.q0 * ahrs.q3;

		const float q11 = ahrs.q1 * ahrs.q1;
		const float q22 = ahrs.q2 * ahrs.q2;
		const float q33 = ahrs.q3 * ahrs.q3;

		const float roll  =  atan2f(2.0f * (q01 + q23), 1.0f - 2.0f * (q11 + q22));
		const float pitch =  asinf( 2.0f * (q02 - q13));
		const float yaw   =  atan2f(2.0f * (q03 + q12), 1.0f - 2.0f * (q22 + q33));

		TheGlobalData.EulerRoll  = roll;
		TheGlobalData.EulerPitch = pitch;
		TheGlobalData.EulerYaw   = yaw;

		data.Roll  = roll;
		data.Pitch = pitch;
		data.Yaw   = yaw;

		// Post to commander unit
		xQueueOverwrite(TheIMUDataQueue, &data);
		TheLedInfo->B(false);
		vTaskDelay(10); // 100Hz.
    }
    vTaskDelete(NULL);
}
