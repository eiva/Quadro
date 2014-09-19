
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
#include "mpu9250.h"
#include "Helpers.h"



int maintestimu(void)
{
	SystemInit();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	LedInfo *info = new LedInfo();
	info->RGBY(true, true, true, true);

	Button *button = new Button(GPIOA, GPIO_Pin_0);
	while (!button->GetState());
	while (button->GetState());
	info->Off();


	Port* mpuCSN = new Port(GPIOB, GPIO_Pin_12);
	mpuCSN->High();

	SpiInterface* spi2 = new SpiInterface(SPI2);


	Logger *logger = new Logger(info, button);

	Mpu9250 mpu(spi2, mpuCSN);
	if (!mpu.Check())
	{
		info->R(true);
		while(1);
	}
	info->G(true);

	// Read: https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino
	uint8_t ReadBuf[14];
	while(1){
		mpu.Read(ReadBuf);
		const float aRes = 4.0f/32768.0f;

		float AX = aRes*(float)Byte16ToInt16(ReadBuf[0],  ReadBuf[1]);  // Acc.X
		float AY = aRes*(float)Byte16ToInt16(ReadBuf[2],  ReadBuf[3]);  // Acc.Y
		float AZ = aRes*(float)Byte16ToInt16(ReadBuf[4],  ReadBuf[5]);  // Acc.Z
		float GX = (float)Byte16ToInt16(ReadBuf[8],  ReadBuf[9]);  // Gyr.X
		float GY = (float)Byte16ToInt16(ReadBuf[10], ReadBuf[11]); // Gyr.Y
		float GZ = (float)Byte16ToInt16(ReadBuf[12], ReadBuf[13]); // Gyr.Z
	}
}
