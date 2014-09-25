#include "stm32f4xx_conf.h"
#include "mavlink.h"
#include "GlobalData.h"
#include "parameters.h"
#include "helpers.h"

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

// Mavlink Parameters Protocol

volatile uint16_t CurrentParameterSent = 0; ///< Currnt ID of sent parameter. If it is 0 - all parameters from 0 to MAX will be passed.

volatile uint16_t CurrentParameterReRead = 0; ///< Currnt ID of requested to read parameter.

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

uint16_t updateParameter(const char* name, float val, uint8_t paramType); ///< Update parameter value, return parameter index.

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
    		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    			{
    				// Start sending parameters
    				CurrentParameterSent = 0;
    			}
    			break;
    		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    		{
    			mavlink_param_request_read_t read;
    			mavlink_msg_param_request_read_decode(&msgRx, &read);
    			CurrentParameterReRead = read.param_index;
    		}
    			break;
    		case MAVLINK_MSG_ID_PARAM_SET:
    		{
    				mavlink_param_set_t set;
    				mavlink_msg_param_set_decode(&msgRx, &set);
    				CurrentParameterReRead = updateParameter(set.param_id, set.param_value, set.param_type);
    		}
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
	int32_t   LastTickExecuted; ///< Last time processing function were called.
	uint16_t   TaskTickDelta;    ///< Period in ticks. Try to set prime number to avoid several processing per call.
	bool      (*Func)();         ///< Processing function.
};

//#define MAX_TASKS 6

bool processHartBeat();
bool processAttitude();
bool processRawSensors();
bool processRCInputRaw();
bool processRCInputScaled();
bool processParameters();

TelemetryTask_t TelemetryTasks[] =
{
		{0, 997,   processHartBeat},
		{0, 769,   processRCInputRaw},
		{0, 379,   processAttitude},
		{0, 211,   processRCInputScaled},
		{0, 199,   processRawSensors},
		{0, 2,     processParameters}
};

/// Init mavlink protocol.
void InitMAVLink()
{
	initUsart();								 ///< Init Comm hardware

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
	mavlink_system.type = MAV_TYPE_QUADROTOR;    ///< This system is an quadrotor

	// Parameters protocol
	CurrentParameterReRead = CurrentParameterSent = GetParamsCount(); ///< Not need to send parameters now.

}

// http://qgroundcontrol.org/mavlink/parameter_protocol

int32_t currentTick = 0; ///< Current telemetry call number. Used as counter.

void ProcessMAVLink()
{
	++currentTick;

	const size_t maxsize = sizeof(TelemetryTasks)/sizeof(TelemetryTask_t);

	for (int taskIndex = 0; taskIndex < maxsize; ++taskIndex)
	{
		TelemetryTask_t task = TelemetryTasks[taskIndex];
		if (currentTick - (int32_t)task.LastTickExecuted < task.TaskTickDelta)
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

uint16_t updateParameter(const char* name, float val, uint8_t paramType)
{
	const uint16_t totalParameters = GetParamsCount();
	param_s param;
	int16_t index = GetParam(param, name);
	if (index == -1)
	{
		return totalParameters;
	}
	switch (param.type)
	{
		case PARAM_UINT32:
		{
			if (paramType != MAV_PARAM_TYPE_UINT32)
			{
				return totalParameters;
			}
			uint32_t temp;
			memcpy(&temp, &val, sizeof(uint32_t));
			*((uint32_t*)param.address) = temp;
		 }
		 break;
		 case PARAM_INT32:
		 {
			 if (paramType != MAV_PARAM_TYPE_INT32)
			 {
				 return totalParameters;
			 }
			 int32_t temp;
			 memcpy(&temp, &val, sizeof(int32_t));
			 *((int32_t*)param.address) = val;
		 }
		 break;
		 case PARAM_FLOAT:
			 if (paramType != MAV_PARAM_TYPE_REAL32)
			 {
				 return totalParameters;
			 }
			 *((float*)param.address) = val;
			 break;
		 default:
			 return false;
	}
	return index;
}

bool sendParameter(uint16_t index)
{
	 param_s param;

	 if (!GetParam(param, index))
	 {
		 return false; // ?
	 }

	 char name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
	 memset(name, 0, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);

	 const uint8_t size = min(MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN-1, strlen(param.name));

	 memcpy(name, param.name, size);

	 float val;

	 MAV_PARAM_TYPE paramType;

	 switch (param.type)
	  {
	 	case PARAM_UINT32:
	 	{
	   		const uint32_t temp = *((uint32_t*)param.address);
	   		memcpy(&val,&temp, sizeof(float));
	   		paramType = MAV_PARAM_TYPE_UINT32;
	 	}
	 		break;
	    case PARAM_INT32:
	    {
	    	const int32_t temp = *((int32_t*)param.address);
	    	memcpy(&val,&temp, sizeof(float));
	    	paramType = MAV_PARAM_TYPE_INT32;
	    }
	   		break;
	    case PARAM_FLOAT:
	    	val = *((float*)param.address);
	    	paramType = MAV_PARAM_TYPE_REAL32;
	   		break;
	 	  default:
	 		  return false;
	  }
	 //static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	 	//					       const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)

	 mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
			 name, val, paramType, GetParamsCount(), index);
}

bool processParameters()
{

 const uint16_t totalParameters = GetParamsCount();
 if (CurrentParameterReRead < totalParameters)
 {
	 if (sendParameter(CurrentParameterReRead))
	 {
		 sendParameter(CurrentParameterReRead);
		 CurrentParameterReRead = totalParameters;
		 return true;
	 }
 }

 if (CurrentParameterSent >= totalParameters)
 {
	 return false;
 }

 ++CurrentParameterSent;

 return sendParameter(CurrentParameterSent-1);
}

bool processHartBeat()
{
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	return true;
}
bool processAttitude()
{
	// Quaternion
	/*mavlink_msg_attitude_quaternion_pack(mavlink_system.sysid, mavlink_system.compid, &msg, TheGlobalData.BootMilliseconds,
					TheGlobalData.AttQ0, TheGlobalData.AttQ1, TheGlobalData.AttQ2, TheGlobalData.AttQ3, 0, 0, 0);*/

	// Euler
	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg, TheGlobalData.BootMilliseconds,
			TheGlobalData.EulerRoll, TheGlobalData.EulerPitch, TheGlobalData.EulerYaw, 0, 0, 0);

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
			0, 0, 0, TheGlobalData.Temperature,
			0x1FF);
	return true;
}

bool processRCInputRaw()
{
	//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
		//					       uint64_t time_usec,
	  // uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)

	mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
			TheGlobalData.BootMilliseconds,
			0,
			TheGlobalData.RT,TheGlobalData.RY, TheGlobalData.RP, TheGlobalData.RR,
			0, 0,0,0,
			255);

	return true;
}
bool processRCInputScaled()
{
	mavlink_msg_rc_channels_scaled_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
			TheGlobalData.BootMilliseconds,
			0,
			(int16_t)(TheGlobalData.ST * 100),(int16_t)(TheGlobalData.SY * 100), (int16_t)(TheGlobalData.SP * 100), (int16_t)(TheGlobalData.SR * 100),
			0, 0,0,0,
			255);
	return true;

}
