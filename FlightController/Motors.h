class Motors{
	TIM_TypeDef* _timer;
	GPIO_TypeDef* _motorsGpioPort;
	uint16_t _gpioMotor1;
	uint16_t _gpioMotor2;
	uint16_t _gpioMotor3;
	uint16_t _gpioMotor4;
	uint16_t _min;
	uint16_t _max;
public:
	Motors():
		_timer(TIM2),
		_motorsGpioPort(GPIOA),
		_gpioMotor1(GPIO_Pin_0),
		_gpioMotor2(GPIO_Pin_1),
		_gpioMotor3(GPIO_Pin_2),
		_gpioMotor4(GPIO_Pin_3)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;

		// Configure the GPIO for the timer output
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = (_gpioMotor1 |
		                               _gpioMotor2 |
		                               _gpioMotor3 |
		                               _gpioMotor4);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(_motorsGpioPort, &GPIO_InitStructure);

		_min = 500; // 0.5 ms as position 0;
		_max = 2500; // 2.5 ms as position 180;
		//Timer configuration
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Period = 20000; // Total period = 20ms;
		TIM_TimeBaseStructure.TIM_Prescaler =  SystemCoreClock/1000000; // 1MHz devider - timer ticks with 1MHz
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(_timer, &TIM_TimeBaseStructure);

		//PWM channels configuration (All identical!)
		TIM_OCStructInit(&TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

		TIM_OC1Init(_timer, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(_timer, TIM_OCPreload_Enable);

		TIM_OC2Init(_timer, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(_timer, TIM_OCPreload_Enable);

		TIM_OC3Init(_timer, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(_timer, TIM_OCPreload_Enable);

		TIM_OC4Init(_timer, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(_timer, TIM_OCPreload_Enable);

		//Enable the timer
		TIM_Cmd(_timer, ENABLE);
		//Enable the timer PWM outputs
		TIM_CtrlPWMOutputs(_timer, ENABLE);
	}

	// Input - value from 0 - 1800 = 1/10 degree.
	void SetRatio(const uint16_t m1, const uint16_t m2, const uint16_t m3, const uint16_t m4){
		TIM_SetCompare1(_timer, map(m1, 0, 1800, _min, _max));
		TIM_SetCompare2(_timer, map(m2, 0, 1800, _min, _max));
		TIM_SetCompare3(_timer, map(m3, 0, 1800, _min, _max));
		TIM_SetCompare4(_timer, map(m4, 0, 1800, _min, _max));
	}
};
