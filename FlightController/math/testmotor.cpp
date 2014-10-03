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
#include "Helpers.h"

int main_dis_test_motorts(void)
{
	SystemInit();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	LedInfo * info = new LedInfo();
	Port csn(GPIOA, GPIO_Pin_4);
	csn.High();
	Port ce (GPIOA, GPIO_Pin_3);

	SpiInterface spi(SPI1);

	Nrf24 nrf(&spi, &csn, &ce);

	RadioLink radioLink(&nrf, info);

	Motors motors;
	info->Off();
	while(true)
	{
		if (radioLink.Update())
		{
			int f = map(radioLink.Throttle, 0.0f, 1023.0f, 0.0f, 100.0f);
			motors.SetRatio(f, f, f, f);
			info->G(true);
		}
		else
		{
			info->R(true);
		}
	}
}


