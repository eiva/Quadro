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

bool Arm(CommanderData &o_rCommanderData); // Enter ESC to arming mode. Long term.
bool Stabilize(CommanderData &o_rCommanderData); // Process stabilization mode.
bool CalibrateAccelerometer(const IMUData &i_rImuData, CommanderData &o_rCommanderData); // State of altimeter calibration.

void vTaskCommander (void *pvParameters)
{
	QueueSetMemberHandle_t xActivatedMember;
	RadioLinkData radioData;
	IMUData imuData;
	LogData log;
	CommanderData commanderData;

	static bool isArmed = false;

	static float lastTick = xTaskGetTickCount();

	//(const float kp, const float ki, const float kd, const float intLimit)
	PidObject yawRatePid(0.3f, 0, 0.1f, 1);
	PidObject pitchPid  (2.0f, 10.0f, 0.13f, 10.0f);
	PidObject rollPid   (2.0f, 10.0f, 0.13f, 10.0f);

	static bool imuReady = false; // We need at last one IMU data packet to start work.
	static bool radioReady = false; // We need at last one radio data packet to start work.
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

    	if (!isArmed)
    	{
    		// We can arm only if IMU and radio ready.
    		if (Arm(commanderData))
    		{
    			xQueueOverwrite( TheCommanderDataQueue, &commanderData );
    			continue;
    		}
    		// Arm done.
    		isArmed = true;
    	}

    	TheLedInfo->Y(true);

    	// dT calculation
    	const float currentTick = xTaskGetTickCount();
    	const float dT = (currentTick - lastTick) / 1000.0f; // Seconds
    	lastTick = currentTick;
    	if (dT == 0)
    	{
    		continue; // Skip first iteration.
    	}

    	if (radioData.Throttle <= 2 )
    	{
    		yawRatePid.Reset();
    		pitchPid.Reset();
    		rollPid.Reset();
    		commanderData.Throttle = 0;
    		commanderData.Pitch    = 0;
    		commanderData.Roll     = 0;
    		commanderData.Yaw      = 0;

    		xQueueOverwrite( TheCommanderDataQueue, &commanderData );
    		continue;
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

    	xQueueOverwrite( TheCommanderDataQueue, &commanderData );

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

// Enter ESC to arming mode.
// This is specific for BLHeli ESC flash: it need to be raised to 50% of pwm level and back to 0.
bool Arm(CommanderData &o_rCommanderData)
{
	// TODO: Need to check throttle stick position. if it is not zero - do not arm.
	const int period = 1500;
	static const TickType_t firstTick = xTaskGetTickCount();
	const TickType_t currentTick = xTaskGetTickCount();
	const int delta = currentTick - firstTick;
	static bool down = false;

	o_rCommanderData.Yaw = 0;
	o_rCommanderData.Roll = 0;
	o_rCommanderData.Pitch = 0;

	if (delta  < period) // Prearm - wait ESC init.
	{
		o_rCommanderData.Throttle = 0;
		return true;
	}

	if (delta  > 4*period) // End of arm
	{
		o_rCommanderData.Throttle = 0;
		return false;
	}

	if (delta > 2*period) // Switch up front to down front.
	{
		down = true;
	}

	int step;
	if (delta < 3*period)
	{
		if (down) // From 50% to 0%
		{
			step = 50 - (delta - 3000) / 30;
		}
		else // From 0% to 50%
		{
			step = (delta - 1500) / 30;
		}
	}
	else
	{
		step = 0;
	}

	o_rCommanderData.Throttle = step;

	return true; // Continue arming.
}
