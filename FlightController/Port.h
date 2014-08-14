// Generic port helper.
class Port{
	GPIO_TypeDef* _gpioPort;
	uint16_t _portNumber;
public:
	Port(GPIO_TypeDef* gpioPort, uint16_t portNumber):
		_portNumber(portNumber),
		_gpioPort(gpioPort)
	{
		GPIO_InitTypeDef port;
		GPIO_StructInit(&port);
		port.GPIO_Pin = portNumber;
		port.GPIO_Mode = GPIO_Mode_Out_PP;
		port.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(_gpioPort, &port);
	}
	// True - on. False - off.
	inline void State(bool state){
		if (state)
			GPIO_SetBits(_gpioPort, _portNumber);
		else
			GPIO_ResetBits(_gpioPort, _portNumber);
	}
	inline void High(){
		GPIO_SetBits(_gpioPort, _portNumber);
	}
	inline void Low(){
		GPIO_ResetBits(_gpioPort, _portNumber);
	}
};
