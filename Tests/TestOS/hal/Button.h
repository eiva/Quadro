class Button
{
	GPIO_TypeDef* _gpioPort;
	uint16_t _portNumber;
	public:
	Button(GPIO_TypeDef* gpioPort, uint16_t portNumber):
			_portNumber(portNumber),
			_gpioPort(gpioPort)
		{
			GPIO_InitTypeDef port;
			GPIO_StructInit(&port);
		    port.GPIO_Mode = GPIO_Mode_IN;
		    port.GPIO_Pin = portNumber;
		    port.GPIO_Speed = GPIO_Speed_2MHz;
		    port.GPIO_PuPd = GPIO_PuPd_NOPULL;
		    GPIO_Init(gpioPort, &port);
		}
		// True - on. False - off.
		inline bool GetState(){
			return GPIO_ReadInputDataBit(_gpioPort, _portNumber) == SET;
		}
};
