#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include "Utils.h"
#include "Port.h"
#include "LedInfo.h"
#include "Motors.h"

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


	Motors m;
	LedInfo leds;

	int d = 1;
	int step = 5;
	int max = 1800;
	int min = 0;
	int r = min;
	leds.W(true);
	while(true){

		m.SetRatio(r, r, max, min);
		Delay(10);

		r += d;
		if (r >= max - 1){
			d = -step;
			leds.RGBW(true, false, false, false);
		}
		if (r <= min + 1){
			d = step;
			leds.RGBW(false, true, false, false);
		}
	}
}


