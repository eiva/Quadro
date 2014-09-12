#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "system_stm32f4xx.h"
#include "LedInfo.h"
#include "Port.h"
#include "SpiInterface.h"
#include "Nrf24.h"
#include "Button.h"

/*
int mainCheckNRF(void)
{
	SystemInit();



	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


	LedInfo info;
	info.RGBY(true, true, true, true);
	Button button(GPIOA, GPIO_Pin_0);
	while (!button.GetState());
	info.Off();

    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE);


	Port csn(GPIOA, GPIO_Pin_4);
	csn.High();
	Port ce (GPIOA, GPIO_Pin_3);

	SpiInterface spi(SPI1);

	Nrf24 nrf(spi, csn, ce);

	while(1)
	{

		info.B(true);
		if (nrf.Check())
		{
			info.G(true);
		}
		else
		{
			info.R(true);
		}
		info.B(false);
	}

	/*
	 * 	SpiInterface spi(SPI1);

	Port csn(GPIOA, GPIO_Pin_4);
	Port ce (GPIOA, GPIO_Pin_3);

	Nrf24 nrf(spi, csn, ce);

	RadioLink radioLink(nrf, info);

	while(1)
	{
	info.B(true);
	if (radioLink.Update())
	{
		info.G(true);
		info.R(false);
	}
	else
	{
		info.R(true);
		info.G(false);
	}
	info.B(false);
	}


	while(1);
}
*/
