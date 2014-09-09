#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

#include "Motors.h"


Motors::Motors() : _min(25), _max(250)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE);

	GPIO_InitTypeDef port;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_OType = GPIO_OType_PP;
    port.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    port.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOE,&port);

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);

    // Timer configuration
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 2000; // Total period = 20ms;
    TIM_TimeBaseStructure.TIM_Prescaler =  SystemCoreClock/1000000; // 1MHz devider - timer ticks with 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Enable the timer
	TIM_Cmd(TIM1, ENABLE);
	// Enable the timer PWM outputs
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare2(TIM1, 0);
	TIM_SetCompare3(TIM1, 0);
	TIM_SetCompare4(TIM1, 0);

}

// Input - value from 0 - 1800 = 1/10 degree.
	void SetRatio(const uint16_t m1, const uint16_t m2, const uint16_t m3, const uint16_t m4){
		//TIM_SetCompare1(TIM1, map(m1, 0, 1800, _min, _max));
		//TIM_SetCompare2(TIM1, map(m2, 0, 1800, _min, _max));
		//TIM_SetCompare3(TIM1, map(m3, 0, 1800, _min, _max));
		//TIM_SetCompare4(TIM1, map(m4, 0, 1800, _min, _max));
	}
