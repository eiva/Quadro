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

int mainTESTMOTOR(void)
{
	SystemInit();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/*
	 * 	Port csn(GPIOA, GPIO_Pin_4);
	csn.High();
	Port ce (GPIOA, GPIO_Pin_3);

	SpiInterface spi(SPI1);

	Nrf24 nrf(spi, csn, ce);

	RadioLink radioLink(nrf, info);

	Motors motors;
	RadioLinkData radioData;
	while(true)
	{
		if (radioLink.Update(radioData))
		{
			motors.SetRatio(radioData.Throttle, radioData.Throttle, radioData.Throttle, radioData.Throttle);
			info.Y(true);
		}
		else
		{
			info.R(true);
		}
	}
	 */

	/*
	LedInfo info;
	info.RGBY(true, true, true, true);
	Button button(GPIOA, GPIO_Pin_0);
	while (!button.GetState());
	info.Off();


	Motors motors;

    info.RGBY(true, true, false, false);
	while(1);
	*/
}


