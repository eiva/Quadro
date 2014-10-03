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
	CommanderData commanderData;

	float lastTick = xTaskGetTickCount();

	//(const float kp, const float ki, const float kd, const float intLimit)
	PidObject yawRatePid(0.3f, 0, 0.1f, 1);
	PidObject pitchPid  (3.5f, 0.0f, 0.0f, 0.0f);
	PidObject rollPid   (3.5f, 0.0f, 0.0f, 0.0f);

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
    		continue;
    	}

    	if (!imuReady || !radioReady)
    	{
    		// Wait for first complete data.
    		continue;
    	}

    	TheLedInfo->Y(true);

    	if (radioData.Throttle <=1 )
    	{
    		TheLedInfo->Y(false);
    		motors.SetRatio(0, 0, 0, 0);
    		continue;
    	}

    	// dT calculation
    	const float currentTick = xTaskGetTickCount();
    	const float dT = (currentTick - lastTick) / 1000.0f; // Seconds
    	lastTick = currentTick;
    	if (dT == 0)
    	{
    		continue; // Skip first iteration.
    	}

    	// Convert radio to angles
    	const float angle_max = radians(30.0f);
    	const float yawRateDesired =  map(radioData.Yaw,   -100.0f, 100.0f, -angle_max, angle_max); // [-angle_max, angle_max] radians
    	const float pitchDesired   = -map(radioData.Pitch, -100.0f, 100.0f, -angle_max, angle_max); // [-angle_max, angle_max] radians
    	const float rollDesired    =  map(radioData.Roll,  -100.0f, 100.0f, -angle_max, angle_max); // [-angle_max, angle_max] radians

    	const float throttleDesired = map(radioData.Throttle, 0.0f, 100.0f, 10.0f, 100.0f); // [10, 100] percent of motor power

    	// Feed PID regulators
    	const float yawCorrection   = yawRatePid.Update( yawRateDesired, imuData.Yaw,   dT, true); // radians
    	const float pitchCorrection = pitchPid  .Update( pitchDesired,   imuData.Pitch, dT, true); // radians
    	const float rollCorrection  = rollPid   .Update( rollDesired,    imuData.Roll,  dT, true); // radians

    	// Calculate motors power
    	const float pm = 30.0f * throttleDesired / 100.0f; // Multiplier for Pitch
    	const float rm = 30.0f * throttleDesired / 100.0f; // Multiplier for Roll
    	const float ym = 5.0f;  // Multiplier for Yaw

    	const float pitchK = pitchCorrection * pm; // [-1, 1] pitch correction
    	const float rollK  = rollCorrection  * rm;  // [-1, 1] roll correction
    	const float yawK   = cosf(yawCorrection);   // [-1, 1] yaw correction

    	commanderData.Throttle = throttleDesired;
    	commanderData.Pitch    = pitchK;
    	commanderData.Roll     = rollK;
    	commanderData.Yaw      = yawK;

    	xQueueOverwrite( TheRadioCommandsQueue, &commanderData );

    	TheGlobalData.DBG_PID_YAW   = yawCorrection;
    	TheGlobalData.DBG_PID_PITCH = pitchCorrection;
    	TheGlobalData.DBG_PID_ROLL  = rollCorrection;

    	TheGlobalData.DBG_CO_YAW    = yawK;
    	TheGlobalData.DBG_CO_PITCH  = pitchK;
    	TheGlobalData.DBG_CO_ROLL   = rollK;

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
