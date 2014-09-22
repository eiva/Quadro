
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
#include "telemetry.h"
#include "GlobalData.h"
#include "mpu9250.h"
#include "Helpers.h"
#include <math.h>

QueueHandle_t TheRadioCommandsQueue;
QueueHandle_t TheIMUDataQueue;
QueueHandle_t TheLogQueue;
QueueSetHandle_t TheStabilizerQueueSet;

struct IMUData
{
	Vector3 EulierAngles;
};

GlobalData TheGlobalData; // Defined in GlobalData file.


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

    		TheGlobalData.RT = data.Throttle;
    		TheGlobalData.RY = data.Yaw;
    		TheGlobalData.RP = data.Pitch;
    		TheGlobalData.RR = data.Roll;

    		vTaskDelay(20); // 50Hz
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
	Mpu9250 *mpu = (Mpu9250*)pvParameters;
	float lastTick = xTaskGetTickCount();
	uint8_t ReadBuf[21];
	while(1)
    {
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

		const float q1s = ahrs.q1 * ahrs.q1;
		const float q2s = ahrs.q2 * ahrs.q2;
		const float q3s = ahrs.q3 * ahrs.q3;

		const float roll  =  atan2f(q23 + q01, 0.5f - (q1s + q2s));
		const float pitch = -asinf(-2.0f * (q13 + q02));
		const float yaw   =  atan2f(q12 + q03, 0.5f - (q2s + q3s));

		TheGlobalData.EulerRoll = roll;
		TheGlobalData.EulerPitch = pitch;
		TheGlobalData.EulerYaw = yaw;

		data.EulierAngles.X = roll;
		data.EulierAngles.Y = pitch;
		data.EulierAngles.Z = yaw;

		// Post to commander unit
		xQueueOverwrite(TheIMUDataQueue, &data);
		vTaskDelay(10); // 100Hz.
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

    	// Testing telemetry.
    	//xQueueOverwrite( TheLogQueue, &log );
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
		//xQueueReceive(TheLogQueue, &data, portMAX_DELAY  );
		//logger->Log(data); // Extra slow!
		ProcessMAVLink();
		vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

int main(void)
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

	InitMAVLink();

	Port* csn = new Port(GPIOA, GPIO_Pin_4);
	csn->High();
	Port* ce = new Port(GPIOA, GPIO_Pin_3);

	SpiInterface* spi = new SpiInterface(SPI1);

	Nrf24* nrf = new Nrf24(spi, csn, ce);

	RadioLink *radioLink = new RadioLink(nrf, info);

	Port* mpuCSN = new Port(GPIOB, GPIO_Pin_12);
	mpuCSN->High();

	SpiInterface* spi2 = new SpiInterface(SPI2);

	Mpu9250 *mpu = new Mpu9250(spi2, mpuCSN);
	if (!mpu->Check())
	{
		info->R(true);
		while(1);
	}

	Logger *logger = new Logger(info, button);

	vFreeRTOSInitAll();
    xTaskCreate(vTaskRFReciever, (char*)"nRF", configMINIMAL_STACK_SIZE, radioLink, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vTaskIMUProcessor,(char*)"STB", configMINIMAL_STACK_SIZE, mpu, tskIDLE_PRIORITY + 2, NULL);
    //xTaskCreate(vTaskCommander,  (char*)"CMD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
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
