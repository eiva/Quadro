#include "stm32f4xx_conf.h"
#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "RadioLink.h"
#include "GlobalData.h"
#include "Helpers.h"
#include "parameters.h"

#include "RFReciever.h"

uint32_t param_rc1_dz   = 0;     // 0 - 200
uint32_t param_rc1_max  = 1824;  // 800 - 2200
uint32_t param_rc1_min  = 800;   // 800 - 2200
int32_t  param_rc1_rev  = 1;     // -1 = reversed 1 = normal
uint32_t param_rc1_trim = 1500;  // 800 - 220

uint32_t param_rc2_dz   = 0;     // 0 - 200
uint32_t param_rc2_max  = 2200;  // 800 - 2200
uint32_t param_rc2_min  = 800;   // 800 - 2200
int32_t  param_rc2_rev  = 1;     // -1 = reversed 1 = normal
uint32_t param_rc2_trim = 1500;  // 800 - 220

uint32_t param_rc3_dz   = 0;     // 0 - 200
uint32_t param_rc3_max  = 2200;  // 800 - 2200
uint32_t param_rc3_min  = 800;   // 800 - 2200
int32_t  param_rc3_rev  = 1;     // -1 = reversed 1 = normal
uint32_t param_rc3_trim = 1500;  // 800 - 220

uint32_t param_rc4_dz   = 0;     // 0 - 200
uint32_t param_rc4_max  = 2200;  // 800 - 2200
uint32_t param_rc4_min  = 800;   // 800 - 2200
int32_t  param_rc4_rev  = 1;     // -1 = reversed 1 = normal
uint32_t param_rc4_trim = 1500;  // 800 - 220

#define TOV(x) (800.0f + (x)) // From RF (~0 - ~ 1024) to PWM (~800 - ~ 2500)

#define TOA(v, i) (TOV(radioLink->v) - param_rc##i##_trim) > 0 ? \
		100.0f * ((TOV(radioLink->v) - param_rc##i##_trim) / (param_rc##i##_max - param_rc##i##_trim)) \
        :\
         -100.0f * ((TOV(radioLink->v) - param_rc##i##_trim) / (param_rc##i##_trim - param_rc##i##_min))

void vTaskRFReciever (void *pvParameters)
{
	const uint16_t minoffset = 800;
	RadioLinkData data;
	RadioLink *radioLink = (RadioLink*)pvParameters;
    while(1)
    {
    	if (radioLink->Update())
    	{
    		// For telemetry.
    		TheGlobalData.RT = (uint16_t)TOV(radioLink->Throttle);
    		TheGlobalData.RY = (uint16_t)TOV(radioLink->Yaw);
    		TheGlobalData.RP = (uint16_t)TOV(radioLink->Pitch);
    		TheGlobalData.RR = (uint16_t)TOV(radioLink->Roll);


    		// Cailibration
    		data.Throttle = (param_rc1_min - TOV(radioLink->Throttle))/param_rc1_max;
    		data.Yaw   = TOA(Yaw, 2);
    		data.Pitch = TOA(Pitch, 3);
    		data.Roll  = TOA(Roll, 4);

    		xQueueOverwrite( TheRadioCommandsQueue, &data );

    		vTaskDelay(20); // 50Hz
    	}
    	else
    	{
    		vTaskDelay(1);
    	}
    }
    vTaskDelete(NULL);
}

// Constant just for QGC compartability
uint32_t param_rc_speed = 60; // Hz
uint32_t param_rcmap_pitch = 2;
uint32_t param_rcmap_roll = 1;
uint32_t param_rcmap_throttle = 3;
uint32_t param_rcmap_yaw = 4;

PARAM_GROUP_START(RFParameters)

	PARAM_ADD(PARAM_UINT32, RC_SPEED, &param_rc_speed)

	PARAM_ADD(PARAM_UINT32, RC1_DZ, &param_rc1_dz)
	PARAM_ADD(PARAM_UINT32, RC1_MAX, &param_rc1_max)
	PARAM_ADD(PARAM_UINT32, RC1_MIN, &param_rc1_min)
	PARAM_ADD(PARAM_INT32,  RC1_REV, &param_rc1_rev)
	PARAM_ADD(PARAM_UINT32, RC1_TRIM, &param_rc1_trim)

	PARAM_ADD(PARAM_UINT32, RC2_DZ,   &param_rc2_dz)
	PARAM_ADD(PARAM_UINT32, RC2_MAX,  &param_rc2_max)
	PARAM_ADD(PARAM_UINT32, RC2_MIN,  &param_rc2_min)
	PARAM_ADD(PARAM_INT32,  RC2_REV,  &param_rc2_rev)
	PARAM_ADD(PARAM_UINT32, RC2_TRIM, &param_rc2_trim)

	PARAM_ADD(PARAM_UINT32, RC3_DZ,   &param_rc3_dz)
	PARAM_ADD(PARAM_UINT32, RC3_MAX,  &param_rc3_max)
	PARAM_ADD(PARAM_UINT32, RC3_MIN,  &param_rc3_min)
	PARAM_ADD(PARAM_INT32,  RC3_REV,  &param_rc3_rev)
	PARAM_ADD(PARAM_UINT32, RC3_TRIM, &param_rc3_trim)

	PARAM_ADD(PARAM_UINT32, RC4_DZ,   &param_rc4_dz)
	PARAM_ADD(PARAM_UINT32, RC4_MAX,  &param_rc4_max)
	PARAM_ADD(PARAM_UINT32, RC4_MIN,  &param_rc4_min)
	PARAM_ADD(PARAM_INT32,  RC4_REV,  &param_rc4_rev)
	PARAM_ADD(PARAM_UINT32, RC4_TRIM, &param_rc4_trim)

	PARAM_ADD(PARAM_UINT32, RCMAP_PITCH,    &param_rcmap_pitch)
	PARAM_ADD(PARAM_UINT32, RCMAP_ROLL,     &param_rcmap_roll)
	PARAM_ADD(PARAM_UINT32, RCMAP_THROTTLE, &param_rcmap_throttle)
	PARAM_ADD(PARAM_UINT32, RCMAP_YAW,      &param_rcmap_yaw)
PARAM_GROUP_STOP(TEST_PARAMS)
