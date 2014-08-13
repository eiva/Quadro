class ADCPort{
	GPIO_TypeDef* _gpioPort;
	uint16_t _portNumber;
	uint16_t _adcPort;
public:
	ADCPort(GPIO_TypeDef *gpioPort, uint16_t gpioPin, uint16_t adcChannel):
		_portNumber(gpioPin),
		_gpioPort(gpioPort),
		_adcPort(adcChannel)
	{
		GPIO_InitTypeDef port;
		GPIO_StructInit(&port);
		port.GPIO_Pin = _portNumber;
		port.GPIO_Mode = GPIO_Mode_AIN;
		port.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(_gpioPort, &port);

		ADC_InitTypeDef ADC_InitStructure;
		ADC_StructInit(&ADC_InitStructure);
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ENABLE;      // we work in continuous sampling mode
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;

		ADC_RegularChannelConfig(ADC1,_adcPort, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
		ADC_Init ( ADC1, &ADC_InitStructure);   //set config of ADC1

		// enable ADC
		ADC_Cmd (ADC1,ENABLE);  //enable ADC1

		//      ADC calibration (optional, but recommended at power on)
		ADC_ResetCalibration(ADC1);     // Reset previous calibration
		while(ADC_GetResetCalibrationStatus(ADC1));
		ADC_StartCalibration(ADC1);     // Start new calibration (ADC must be off at that time)
		while(ADC_GetCalibrationStatus(ADC1));

		// start conversion
		ADC_Cmd (ADC1,ENABLE);  //enable ADC1
		ADC_SoftwareStartConvCmd(ADC1, ENABLE); // start conversion (will be endless as we are in continuous mode)
	}

	uint16_t Read()	{
		// adc is in free run, and we get the value asynchronously, this is not a really nice way of doing, but it work!
		//return ADC_GetConversionValue(ADC1) ; // value from 0 to 4095
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);					// start ONE conversion
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);	// wait end of conversion
		return ADC_GetConversionValue(ADC1);
	}
	/* possible change :
	     * ADC_ContinuousConvMode = DISABLE
	     * then on the infinite loop, something like :
	     *
	     *  ADC_SoftwareStartConvCmd(ADC1, ENABLE);					// start ONE conversion
	     *  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);	// wait end of conversion
	     *  j = ADC_GetConversionValue(ADC1) * 500;					// get value
	     *
	     */
};
