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
#include "telemetry.h"
#include "GlobalData.h"
#include "mpu9250.h"
#include "Helpers.h"
#include "parameters.h"
#include "IMUProcessor.h"
#include "RFReciever.h"
#include "Commander.h"
#include "MotorsMatrix.h"

QueueHandle_t TheRadioCommandsQueue;
QueueHandle_t TheIMUDataQueue;
QueueHandle_t TheLogQueue;
QueueSetHandle_t TheStabilizerQueueSet;
QueueHandle_t TheCommanderDataQueue;


GlobalData TheGlobalData; // Defined in GlobalData file.

LedInfo *TheLedInfo;


/*******************************************************************/
void vFreeRTOSInitAll()
{
	TheRadioCommandsQueue = xQueueCreate(1, sizeof(RadioLinkData));

	TheIMUDataQueue = xQueueCreate(1, sizeof(IMUData));

	TheLogQueue = xQueueCreate(1, sizeof(LogData));

	TheCommanderDataQueue = xQueueCreate(1, sizeof(CommanderData));

	TheStabilizerQueueSet = xQueueCreateSet(1+5);
	xQueueAddToSet( TheRadioCommandsQueue, TheStabilizerQueueSet );
	xQueueAddToSet( TheIMUDataQueue, TheStabilizerQueueSet );
}

void vTaskDataLogger (void *pvParameters)
{
	Logger *logger = (Logger*)pvParameters;
    LogData data;
	while(1)
    {
		TheLedInfo->R(true);
		//xQueueReceive(TheLogQueue, &data, portMAX_DELAY  );
		//logger->Log(data); // Extra slow!
		ProcessMAVLink();
		TheLedInfo->R(false);
		vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

int maindisabled(void)
{
	SystemInit();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	TheLedInfo = new LedInfo();
	TheLedInfo->RGBY(true, true, true, true);

	Button *button = new Button(GPIOA, GPIO_Pin_0);
	//while (!button->GetState());
	//while (button->GetState());
	TheLedInfo->Off();

	InitParams();

	InitMAV3Link();

	Port* csn = new Port(GPIOA, GPIO_Pin_4);
	csn->High();
	Port* ce = new Port(GPIOA, GPIO_Pin_3);

	SpiInterface* spi = new SpiInterface(SPI1);

	Nrf24* nrf = new Nrf24(spi, csn, ce);

	RadioLink *radioLink = new RadioLink(nrf, TheLedInfo);

	Port* mpuCSN = new Port(GPIOB, GPIO_Pin_12);
	mpuCSN->High();

	SpiInterface* spi2 = new SpiInterface(SPI2);

	Mpu9250 *mpu = new Mpu9250(spi2, mpuCSN);
	if (!mpu->Check())
	{
		TheLedInfo->R(true);
		while(1);
	}

	Logger *logger = new Logger(TheLedInfo, button);

	vFreeRTOSInitAll();
    xTaskCreate(vTaskRFReciever,  (char*)"nRF", configMINIMAL_STACK_SIZE, radioLink, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vTaskIMUProcessor,(char*)"STB", configMINIMAL_STACK_SIZE, mpu,       tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vTaskCommander,   (char*)"CMD", configMINIMAL_STACK_SIZE, NULL,      tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vTaskMotorMatrix, (char*)"MTR", configMINIMAL_STACK_SIZE, NULL,      tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskDataLogger,  (char*)"LOG", configMINIMAL_STACK_SIZE, logger,    tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();

    // We should not be here.
	while(1);
}
