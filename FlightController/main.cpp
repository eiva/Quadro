#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_spi.h>
#include <misc.h>
#include "Utils.h"
#include "Port.h"
#include "LedInfo.h"
#include "Motors.h"
#include "ADCPort.h"
#include "SpiInterface.h"
#include "Nrg24.h"
#include "RadioLink.h"
/*
 * Notepad:
 * SystemCoreClock = 72000000
 */




int main(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	SysTick_Config(SystemCoreClock/1000);

	LedInfo leds;

	/*
	 *  Radio link test.
	SpiInterface spi(SPI1, GPIOA, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_5);
	RadioLink channel(spi, leds);
	leds.G(true);
	Motors motor;

	while(true){
		if (channel.Update())
		{
			int val1 = map(channel.Throttle,0,1024,0,1800);
			int val2 = map(channel.Yaw,0,1024,0,1800);
			int val3 = map(channel.Pitch,0,1024,0,1800);
			int val4 = map(channel.Roll,0,1024,0,1800);
			motor.SetRatio(val1, val2, val3, val4);
			leds.B(true);
			leds.R(false);
		}
		else
		{
			leds.R(true);
		}
	}
	*/

	/*
	 * ADC Test
	ADCPort adc(GPIOB, GPIO_Pin_0, ADC_Channel_8);

	while(true){
		Delay(50);

		uint16_t value = adc.Read();
		uint16_t motorVal = map(value, 0, 4095, 10, 1800);
		motor.SetRatio(motorVal, motorVal, 0, 1800);
		if (value <= 1024){
			leds.RGBW(true, false, false, false);
		}else if (value <=2*1024){
			leds.RGBW(false, true, false, false);
		}else if (value <=3*1024){
			leds.RGBW(false, false, true, false);
		}else{
			leds.RGBW(false, false, false, true);
		}
	}*/
	/*
	 * Servo test
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
	}*/
}


