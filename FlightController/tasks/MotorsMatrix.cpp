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
	while(1)
	{
		xQueueReceive(TheCommanderDataQueue, &commanderData, portMAX_DELAY);
		const float currentTick = xTaskGetTickCount();

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

    	vTaskDelay(20); // 50Hz
	}
}
