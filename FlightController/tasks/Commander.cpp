#include "stm32f4xx_conf.h"
#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "Helpers.h"
#include "parameters.h"
#include "IMUProcessor.h"
#include "RFReciever.h"
#include "logger.h"
#include "Motors.h"
#include "Commander.h"
#include "PidObject.h"
#include "GlobalData.h"
#include "LedInfo.h"

extern LedInfo* TheLedInfo;

void vTaskCommander (void *pvParameters)
{
	QueueSetMemberHandle_t xActivatedMember;
	RadioLinkData radioData;
	IMUData imuData;
	Motors motors;
	LogData log;

	float lastTick = xTaskGetTickCount();

	//(const float kp, const float ki, const float kd, const float intLimit)
	PidObject yawRatePid(1.3f, 0, 0.1f, 1);
	PidObject pitchPid(1.8f, 0, 0, 1);
	PidObject rollPid(1.8f, 0, 0, 1);

	bool imuReady = false;
	bool radioReady = false;
    while(1)
    {
    	xActivatedMember = xQueueSelectFromSet(TheStabilizerQueueSet, 10);
    	if (xActivatedMember == TheRadioCommandsQueue)
    	{
    		xQueueReceive(TheRadioCommandsQueue, &radioData, 0 );
    		radioReady = true;
    	}
    	if (xActivatedMember == TheIMUDataQueue)
    	{
    		xQueueReceive(TheIMUDataQueue, &imuData, 0 );
    		imuReady = true;
    	}
    	else
    	{
    		// failure!
    		lastTick = xTaskGetTickCount();
    		continue;
    	}
    	if (!imuReady || !radioReady)
    	{
    		// Wait for first complete data.
    		lastTick = xTaskGetTickCount();
    		continue;
    	}

    	TheLedInfo->Y(true);

    	// dT calculation
    	const float currentTick = xTaskGetTickCount();
    	const float dT = (currentTick - lastTick) / 1000.0f; // Seconds
    	lastTick = currentTick;

    	// Convert radio to angles
    	const float angle_max = radians(30.0f);
    	const float yawRateDesired = map(radioData.Yaw,   -100.0f, 100.0f, -angle_max, angle_max); // [-angle_max, angle_max] radians
    	const float pitchDesired   = map(radioData.Pitch, -100.0f, 100.0f, -angle_max, angle_max); // [-angle_max, angle_max] radians
    	const float rollDesired    = map(radioData.Roll,  -100.0f, 100.0f, -angle_max, angle_max); // [-angle_max, angle_max] radians

    	const float throttleDesired = radioData.Throttle; // [0, 100] percent of motor power

    	// Feed PID regulators
    	const float yawCorrection   = yawRatePid.Update(yawRateDesired, radioData.Yaw,   dT, true); // radians
    	const float pitchCorrection = pitchPid.Update(  pitchDesired,   radioData.Pitch, dT, true); // radians
    	const float rollCorrection  = rollPid.Update(   rollDesired,    radioData.Roll,  dT, true); // radians

    	// Calculate motors power
    	const float pm = 10.0f; // Multiplier for Pitch
    	const float rm = 10.0f; // Multiplier for Roll
    	const float ym = 5.0f;  // Multiplier for Yaw

    	const float pitchK = cosf(pitchCorrection); // [-1, 1] pitch correction
    	const float rollK  = cosf(rollCorrection);  // [-1, 1] roll correction
    	const float yawK   = cosf(yawCorrection);   // [-1, 1] yaw correction

#define MIX(R, P, Y) throttleDesired

    	float m1 = throttleDesired;
    	float m2 = throttleDesired;
    	float m3 = throttleDesired;
    	float m4 = throttleDesired;


    	// Normalize motors power

    	// Set motor power
    	motors.SetRatio(m1, m2, m3, m4);

    	// Telemetry
    	TheGlobalData.MO1 = m1 * 100.0f;
    	TheGlobalData.MO2 = m2 * 100.0f;
    	TheGlobalData.MO3 = m3 * 100.0f;
    	TheGlobalData.MO4 = m4 * 100.0f;

    	TheLedInfo->Y(false);

    	/*
    	log.Timer = xTaskGetTickCount();
    	log.InputThrottle = radioData.Throttle;
    	log.InputYaw = radioData.Yaw;
    	log.InputPitch = radioData.Pitch;
    	log.InputRoll = radioData.Roll;
    	log.Yaw = imuData.EulierAngles.Z;
    	log.Roll = imuData.EulierAngles.Y;
    	log.Pitch = imuData.EulierAngles.X;
    	*/
    	// Testing telemetry.
    	//xQueueOverwrite( TheLogQueue, &log );
    	// Process Data!
    }
    vTaskDelete(NULL);
}
