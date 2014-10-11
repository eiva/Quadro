#include "stm32f4xx_conf.h"
#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "Helpers.h"
#include "parameters.h"
#include "logger.h"
#include "Motors.h"
#include "Commander.h"
#include "GlobalData.h"
#include "LedInfo.h"
#include "MotorsMatrix.h"


void vTaskMotorMatrix(void *pvParameters)
{
	Motors motors;
	CommanderData commanderData;

	const TickType_t frequency = 4; // 4ms interval.
	while(1)
	{
		TickType_t currentTick = xTaskGetTickCount();
		xQueueReceive(TheCommanderDataQueue, &commanderData, portMAX_DELAY);

    	if (commanderData.Throttle == 0 )
    	{
    		motors.SetRatio(0, 0, 0, 0);
    		continue;
    	}

#define MIXRP(P, R) commanderData.Throttle + (R) * commanderData.Roll + (P) * commanderData.Pitch

    	const float m1 = MIXRP( 1,-1);
    	const float m2 = MIXRP(-1, 1);
    	const float m3 = MIXRP( 1, 1);
    	const float m4 = MIXRP(-1,-1);

    	// Normalize motors power

    	// Set motor power
    	motors.SetRatio(m1, m2, m3, m4);

    	// Telemetry
    	TheGlobalData.MOUpdateTime = currentTick;
    	TheGlobalData.MO1 = m1;
    	TheGlobalData.MO2 = m2;
    	TheGlobalData.MO3 = m3;
    	TheGlobalData.MO4 = m4;

    	vTaskDelayUntil(&currentTick, frequency); // 250Hz
	}
}
