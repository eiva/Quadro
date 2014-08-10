#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include "Utils.h"
#include "Port.h"
#include "LedInfo.h"

/*
 * Notepad
 * SystemCoreClock = 72000000
 */


int main(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	SysTick_Config(SystemCoreClock/1000);

	LedInfo leds;

	while(true){

		leds.RGBW(true, false, false, false);
		Delay(1000);
		leds.RGBW(false, true, false, false);
		Delay(1000);
		leds.RGBW(false, false, true, false);
		Delay(1000);
		leds.RGBW(false, false, false, true);
		Delay(1000);
	}
}


