#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "system_stm32f4xx.h"
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

QueueHandle_t TheRadioCommandsQueue;
QueueHandle_t TheIMUDataQueue;
QueueSetHandle_t TheStabilizerQueueSet;

struct IMUData
{
	uint32_t Time; // Milliseconds?
	Vector3 Gyro;
	Vector3 Accel;
};



/*******************************************************************/
void vFreeRTOSInitAll()
{
	TheRadioCommandsQueue = xQueueCreate(1, sizeof(RadioLinkData));

	TheIMUDataQueue = xQueueCreate(5, sizeof(IMUData));

	TheStabilizerQueueSet = xQueueCreateSet(1+5);
	xQueueAddToSet( TheStabilizerQueueSet, TheRadioCommandsQueue );
	xQueueAddToSet( TheStabilizerQueueSet, TheIMUDataQueue );
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

void vTaskIMUReciever (void *pvParameters)
{
	IMUData data;
	while(1)
    {
		// TODO: read.
		data.Time = xTaskGetTickCount();
		if (!xQueueSend(TheIMUDataQueue, &data, 1))
		{
			// Overflow?
		}
		vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void vTaskStabilizer (void *pvParameters)
{
	QueueSetMemberHandle_t xActivatedMember;
	RadioLinkData radioData;
	IMUData imuData;
	Motors motors;
    while(1)
    {
    	xActivatedMember = xQueueSelectFromSet(TheStabilizerQueueSet, 10);
    	if (xActivatedMember == TheRadioCommandsQueue)
    	{
    		xQueueReceive(TheRadioCommandsQueue, &radioData, 0 );
    		motors.SetRatio(radioData.Throttle, radioData.Throttle, radioData.Throttle, radioData.Throttle);
    	}
    	if (xActivatedMember == TheIMUDataQueue)
    	{
    		xQueueReceive(TheIMUDataQueue, &imuData, 0 );
    	}
    	else
    	{
    		// failure!
    		continue;
    	}
    	// Process Data!
    }
    vTaskDelete(NULL);
}

void vTaskDataLogger (void *pvParameters)
{
    while(1)
    {
    	vTaskDelay(50);
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
	Button button(GPIOA, GPIO_Pin_0);
	while (!button.GetState());
	info->Off();


	Port* csn = new Port(GPIOA, GPIO_Pin_4);
	csn->High();
	Port* ce = new Port(GPIOA, GPIO_Pin_3);

	SpiInterface* spi = new SpiInterface(SPI1);

	Nrf24* nrf = new Nrf24(spi, csn, ce);

	RadioLink *radioLink = new RadioLink(nrf, info);

	vFreeRTOSInitAll();
    xTaskCreate(vTaskRFReciever, (char*)"nRF", configMINIMAL_STACK_SIZE, radioLink, tskIDLE_PRIORITY + 2, NULL);
    //xTaskCreate(vTaskIMUReciever,(char*)"IMU", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskStabilizer, (char*)"STB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    //xTaskCreate(vTaskDataLogger, (char*)"LOG", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();

    // We should not be here.
    info->RGBY(true, true, true, true);
	while(1);
}

extern "C"
{
/*******************************************************************/
void vApplicationIdleHook( void )
{
}



/*******************************************************************/
void vApplicationMallocFailedHook( void )
{
    for( ;; );
}



/*******************************************************************/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    for( ;; );
}



/*******************************************************************/
void vApplicationTickHook( void )
{
}

void _exit(int status){
	while(1);
}

}
