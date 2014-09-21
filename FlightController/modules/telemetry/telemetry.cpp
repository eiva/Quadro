#include "stm32f4xx_conf.h"
#include "mavlink.h"
#include "GlobalData.h"

namespace
{

mavlink_system_t mavlink_system;

// Define the system type, in this case an airplane
uint8_t system_type = MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

// Initialize the required buffers for Transmit
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

// Initialize the required buffers for Recieve
mavlink_message_t msgRx;

mavlink_status_t status;


// HAL: Usart used for comm
USART_TypeDef* _usart = USART1;

}

void initUsart()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	if (_usart == USART1)
	{
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); ///< Enable GPIO clock
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); ///< Enable UART clock

		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		  GPIO_Init(GPIOB, &GPIO_InitStructure);

		  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
		  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	}
	else while(1);

	// Configure the NVIC Preemption Priority Bits
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    // Enable the USART1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Configure USART
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 57600; // TODO: Extract as constant
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(_usart, &USART_InitStructure);

	USART_Cmd(_usart, ENABLE); ///< Enable the USART1

	USART_ITConfig(_usart, USART_IT_RXNE, ENABLE);
}

void sendUsart(const uint8_t *buffer, const uint16_t length)
{
	int i;

	for(i = 0; i < length; ++i)
	{
		// wait until data register is empty
		while( !(_usart->SR & 0x00000040) );
		USART_SendData(_usart, *buffer);
		buffer++;
	}
}

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
    	while( !(_usart->SR & 0x00000040) );
    	USART_SendData(_usart, ch);
    }
}

extern "C" void USART1_IRQHandler(void)
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    uint8_t c = USART_ReceiveData(USART1);

    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msgRx, &status))
    {
    	// Handle message
     	switch(msgRx.msgid)
    	{
     		case MAVLINK_MSG_ID_HEARTBEAT:
    		{
    			// E.g. read GCS heartbeat and go into
                // comm lost mode if timer times out
    		}
    		break;
    		case MAVLINK_MSG_ID_COMMAND_LONG:
    			// EXECUTE ACTION
    		break;
    		default:
    				//Do nothing
    		break;
    	}
    }

  }

  //USART_ClearFlag(USART1, USART_IT_RXNE);
  //USART_ClearFlag(USART1, USART_FLAG_LBD);
  //USART_ClearFlag(USART1, USART_FLAG_TC);
}

struct TelemetryTask_t
{
	uint32_t   LastTickExecuted;
	uint16_t   TaskTickDelta;
	bool      (*Func)();
};

#define MAX_TASKS 3

bool processHartBeat();
bool processAttitude();
bool processRawSensors();

TelemetryTask_t TelemetryTasks[MAX_TASKS] =
{
		{0, 200, processHartBeat},
		{0, 201, processAttitude},
		{0, 11,  processRawSensors}
};

void InitMAVLink()
{
	initUsart();								 ///< Init Comm hardware

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
	mavlink_system.type = MAV_TYPE_QUADROTOR;    ///< This system is an quadrotor
}

// http://qgroundcontrol.org/mavlink/parameter_protocol

int currentTick = 0; ///< Current telemetry call number. Used as counter.

void ProcessMAVLink()
{
	++currentTick;

	for (int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex)
	{
		TelemetryTask_t task = TelemetryTasks[taskIndex];
		if (currentTick - task.LastTickExecuted < task.TaskTickDelta)
		{
			continue;
		}
		task.LastTickExecuted = currentTick;
		if (task.Func())
		{
			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

			sendUsart(buf, len);
		}
	}
}

bool processHartBeat()
{
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	return true;
}
bool processAttitude()
{
	mavlink_msg_attitude_quaternion_pack(mavlink_system.sysid, mavlink_system.compid, &msg, TheGlobalData.BootMilliseconds,
					TheGlobalData.AttQ0, TheGlobalData.AttQ1, TheGlobalData.AttQ2, TheGlobalData.AttQ3, 0, 0, 0);
	return true;
}
bool processRawSensors()
{
	//uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag,
	//float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated)
	mavlink_msg_hil_sensor_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
			TheGlobalData.BootMilliseconds * 1000,
			TheGlobalData.AX, TheGlobalData.AY, TheGlobalData.AZ,
			TheGlobalData.GX, TheGlobalData.GY, TheGlobalData.GZ,
			TheGlobalData.MX, TheGlobalData.MY, TheGlobalData.MZ,
			0, 0, 0, 0,
			0x1FF);
	return true;
}
