
#include "stm32f4xx_conf.h"

#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "LedInfo.h"
#include "Motors.h"
#include "Port.h"
#include "SpiInterface.h"
#include "Nrf24.h"
#include "Button.h"
#include "RadioLink.h"
#include "Vector.h"
#include "logger.h"
#include "stdlib.h"
#include "MadwickAHRS.h"

QueueHandle_t TheRadioCommandsQueue;
QueueHandle_t TheIMUDataQueue;
QueueHandle_t TheLogQueue;
QueueSetHandle_t TheStabilizerQueueSet;

struct IMUData
{
	Vector3 EulierAngles;
};


/*******************************************************************/
void vFreeRTOSInitAll()
{
	TheRadioCommandsQueue = xQueueCreate(1, sizeof(RadioLinkData));

	TheIMUDataQueue = xQueueCreate(1, sizeof(IMUData));

	TheLogQueue = xQueueCreate(1, sizeof(LogData));

	TheStabilizerQueueSet = xQueueCreateSet(1+5);
	xQueueAddToSet( TheRadioCommandsQueue, TheStabilizerQueueSet );
	xQueueAddToSet( TheIMUDataQueue, TheStabilizerQueueSet );
}

/*******************************************************************/
void vTaskRFReciever (void *pvParameters)
{
	RadioLinkData data;
	RadioLink *radioLink = (RadioLink*)pvParameters;
    while(1)
    {
    	if (radioLink->Update(data))
    	{
    		xQueueOverwrite( TheRadioCommandsQueue, &data );
    		vTaskDelay(20);
    	}
    	else
    	{
    		vTaskDelay(1);
    	}
    }
    vTaskDelete(NULL);
}

void vTaskIMUProcessor (void *pvParameters)
{
	IMUData data;
	MadwickAHRS ahrs;
	float lastTick = xTaskGetTickCount();
	while(1)
    {
		// Receive data from IMU;
		// I still dont have one, so just dummies.
		vTaskDelay(2);

		const float currentTick = xTaskGetTickCount();

		float gx = 0, gy = 0, gz = 0;
		float ax = 0, ay = 0, az = -1;
		float mx = 0, my = 0, mz = 0;

		// dT calculation
		const float dT = (currentTick - lastTick) / 1000.0f; // Seconds
		lastTick = currentTick;

		// AHRS update
		ahrs.MadgwickAHRSupdate(dT, gx, gy, gz, ax, ay, az, mx, my, mz);

		// Euler calculation...

		data.EulierAngles.X = ahrs.q0;
		data.EulierAngles.Y = ahrs.q1;
		data.EulierAngles.Z = ahrs.q2;
		// Post to commander unit
		xQueueOverwrite(TheIMUDataQueue, &data);

    }
    vTaskDelete(NULL);
}

void vTaskCommander (void *pvParameters)
{
	QueueSetMemberHandle_t xActivatedMember;
	RadioLinkData radioData;
	IMUData imuData;
	Motors motors;
	LogData log;
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

    	motors.SetRatio(radioData.Throttle, radioData.Yaw, radioData.Pitch, radioData.Roll);

    	log.Timer = xTaskGetTickCount();
    	log.InputThrottle = radioData.Throttle;
    	log.InputYaw = radioData.Yaw;
    	log.InputPitch = radioData.Pitch;
    	log.InputRoll = radioData.Roll;
    	log.Yaw = imuData.EulierAngles.Z;
    	log.Roll = imuData.EulierAngles.Y;
    	log.Pitch = imuData.EulierAngles.X;

    	xQueueOverwrite( TheLogQueue, &log );
    	// Process Data!
    }
    vTaskDelete(NULL);
}

void vTaskDataLogger (void *pvParameters)
{
	Logger *logger = (Logger*)pvParameters;
    LogData data;
	while(1)
    {
		xQueueReceive(TheLogQueue, &data, portMAX_DELAY  );
		logger->Log(data); // Extra slow!

    }
    vTaskDelete(NULL);
}

int maindisabled111(void)
{
	SystemInit();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	LedInfo *info = new LedInfo();
	info->RGBY(true, true, true, true);

	Button *button = new Button(GPIOA, GPIO_Pin_0);
	while (!button->GetState());
	while (button->GetState());
	info->Off();


	Port* csn = new Port(GPIOA, GPIO_Pin_4);
	csn->High();
	Port* ce = new Port(GPIOA, GPIO_Pin_3);

	SpiInterface* spi = new SpiInterface(SPI1);

	Nrf24* nrf = new Nrf24(spi, csn, ce);

	RadioLink *radioLink = new RadioLink(nrf, info);

	Logger *logger = new Logger(info, button);

	vFreeRTOSInitAll();
    xTaskCreate(vTaskRFReciever, (char*)"nRF", configMINIMAL_STACK_SIZE, radioLink, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vTaskIMUProcessor,(char*)"STB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskCommander,  (char*)"CMD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vTaskDataLogger, (char*)"LOG", configMINIMAL_STACK_SIZE, logger, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();

    // We should not be here.
    info->RGBY(true, true, true, true);
	while(1);
}



extern "C"
{


void _exit(int status){
	while(1);
}

}
