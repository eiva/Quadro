#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_spi.h>
#include <misc.h>

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"

#include "Utils.h"

#include "Port.h"
#include "LedInfo.h"
#include "Motors.h"
#include "ADCPort.h"
#include "SpiInterface.h"
#include "mpu9250.h"
#include "Nrg24.h"
#include "RadioLink.h"
#include "Vector.h"
#include "PidObject.h"
#include "mpu9250.h"
/*
 * Notepad:
 * SystemCoreClock = 72000000
 */
class Sensor{
public:
	Vector3 Readings;
	virtual bool Update() = 0;
};

// 6 DOF processor
// Just stub for now.
class SensorProcessor{
	ADCPort *_accel;
	ADCPort *_gyro;
public:
	float Pitch;
	float Roll;

	SensorProcessor(ADCPort *accel, ADCPort* gyro):
		Pitch(0),
		Roll(0),
		_accel(accel),
		_gyro(gyro){}

	bool Update(){
		Pitch = mapDeg(_accel->Read(), 4096, 30.0f);
		Roll  = mapDeg(_gyro->Read(),  4096, 30.0f);
		return true;
	}
};

class Commander{
	RadioLink *_radio;
public:
	Commander(RadioLink *radio):
		_radio(radio){}

	float Throttle;    //    [0-100]%
	float Pitch, Roll; // +- [0-30deg] 30degrees; absolute position.
	float YawRate;     // Rate of Yaw: degrees per second;

	bool Update(){
		if (!_radio->Update()) return false;
		Throttle = _radio->Throttle * 100.0f / 1024.0f;
		YawRate  =  mapDeg(_radio->Yaw,   1024, 5.0f);
		Pitch    =  mapDeg(_radio->Pitch, 1024, 30.0f);
		Roll     =  mapDeg(_radio->Roll,  1024, 30.0f);
		return true;
	}
};

#define PID_ROLL_RATE_KP  0.8f
#define PID_ROLL_RATE_KI  0.01f
#define PID_ROLL_RATE_KD  0.01f
#define PID_ROLL_RATE_INTEGRATION_LIMIT    0.5f

#define PID_PITCH_RATE_KP  0.8f
#define PID_PITCH_RATE_KI  0.01f
#define PID_PITCH_RATE_KD  0.01f
#define PID_PITCH_RATE_INTEGRATION_LIMIT   0.5f

#define PID_YAW_RATE_KP  0.4f
#define PID_YAW_RATE_KI  0.01f
#define PID_YAW_RATE_KD  0.01f
#define PID_YAW_RATE_INTEGRATION_LIMIT     0.5f

class Stabilizer{
	SensorProcessor *_sensorProcessor;
	Commander *_commander;
	PidObject _yawRate, _pitch, _roll;
public:
	Stabilizer(SensorProcessor *sensor, Commander *commander):
		_sensorProcessor(sensor),
		_commander(commander),
		_roll(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, PID_ROLL_RATE_INTEGRATION_LIMIT),
		_pitch(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, PID_PITCH_RATE_INTEGRATION_LIMIT),
		_yawRate(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, PID_YAW_RATE_INTEGRATION_LIMIT)
		{}

	bool Update(){
		_sensorProcessor->Update();
		_commander->Update();
		//const unsigned long mic = Micros();
		//const float dt = (mic - _lastUpdateTime)/1000000.0f;
		const float dt = 0.1f;
		//const float yaw = _yawLimit.Rescale(refPacket.YAW);
		//const float pitch = _pitchLimit.Rescale(refPacket.PTC);
		//const float roll = _rollLimit.Rescale(refPacket.ROL);
		//const float throttle = map(refPacket.THR, _throttleLimit.Min, _throttleLimit.Max, 0, 1024);
		_yawRate.SetDesired(0);//_commander->YawRate);
		_pitch.SetDesired(_commander->Pitch);
		_roll.SetDesired(_commander->Roll);
		const float stabYaw = 0;//_sensorProcessor.Yaw;
		const float stabRoll = _sensorProcessor->Roll;
		const float stabPitch = _sensorProcessor->Pitch;
		Yaw = _yawRate.Update(stabYaw, dt, true);
		Pitch = _pitch.Update(stabPitch, dt, true);
		Roll = _roll.Update(stabRoll, dt, true);
		Throttle = _commander->Throttle / 100.0f;
		return true;
	}
	float Throttle;    // [0-1] float
	float Pitch, Roll; // [-30 -- +30] in Radians.
	float Yaw;         // [~0]
};

class ThrottleDistributor{
public:
	ThrottleDistributor():
		M1(0), M2(0), M3(0), M4(0)
	{}
	void Update(float throttle, float yaw, float pitch, float roll){

		#define MIXPWM(r,p,y) map(1800 * throttle + roll * 1400.0f * r + pitch * 1400.0f * p +  yaw * 1400.0f * y, 0, 8*255, 0, 1800)
		M1 = MIXPWM(-1,+1,-1);
		M1 = MIXPWM(-1,-1,+1);
		M2 = MIXPWM(+1,+1,+1);
		M3 = MIXPWM(+1,-1,-1);
	}

	uint16_t M1, M2, M3, M4; // Motor Power: [0-1800]
};

class Controller{
	Stabilizer *_stabilizer;
	Motors *_motors;
	ThrottleDistributor _distributor;
public:
	Controller(Stabilizer *stabilizer, Motors *motors):
		_stabilizer(stabilizer), _motors(motors)
	{}
	void Update(){
		_stabilizer->Update();
		_distributor.Update(_stabilizer->Throttle, _stabilizer->Yaw, _stabilizer->Pitch, _stabilizer->Roll);
		_motors->SetRatio(_distributor.M1, _distributor.M2, _distributor.M3, _distributor.M4);
	}
};

#define Byte16(hi,lo) (((uint16_t)hi) << 8 | ((uint16_t)lo))

int main(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	EXTI_Configuration(); // USB.

	SysTick_Config(SystemCoreClock/1000);

	USB_Interrupts_Config();

    Set_USBClock();

	LedInfo leds;
	SpiInterface spi1(SPI1, GPIOA, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_5);
	RadioLink channel(&spi1, &leds);
	//SpiInterface spi2(SPI2, GPIOB, GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_13,SPI_CPOL_High,SPI_CPHA_2Edge);
	//Port port(GPIOB, GPIO_Pin_2);
	//Mpu9250 mpu(spi2, port);

	/*while(true)
	{
		Delay(10);
	if (mpu.Check())
		leds.G(true);
	else
		leds.R(true);
	}*/

	uint8_t ReadBuf[14];
	while(true){
		//mpu.Read(ReadBuf);
		/*uint16_t AX = Byte16(ReadBuf[0],  ReadBuf[1]);  // Acc.X
		uint16_t AY = Byte16(ReadBuf[2],  ReadBuf[3]);  // Acc.Y
		uint16_t AZ = Byte16(ReadBuf[4],  ReadBuf[5]);  // Acc.Z
		uint16_t GX = Byte16(ReadBuf[8],  ReadBuf[9]);  // Gyr.X
		uint16_t GY = Byte16(ReadBuf[10], ReadBuf[11]); // Gyr.Y
		uint16_t GZ = Byte16(ReadBuf[12], ReadBuf[13]); // Gyr.Z

		TheReport.SAX = AX;
		TheReport.SAY = AY;
		TheReport.SAZ = AZ;
		TheReport.SGX = GX;
		TheReport.SGY = GY;
		TheReport.SGZ = GZ;
		*/
		if (channel.Update())
		{
			int val1 = map(channel.Throttle,0,1024,0,1800);
			int val2 = map(channel.Yaw,0,1024,0,1800);
			int val3 = map(channel.Pitch,0,1024,0,1800);
			int val4 = map(channel.Roll,0,1024,0,1800);
			TheReport.SAX = val1;
			TheReport.SAY = val2;
			TheReport.SAZ = val3;
			TheReport.SGX = val4;

			//motor.SetRatio(val1, val2, val3, val4);
			leds.B(true);
			leds.R(false);

			if (bDeviceState == CONFIGURED)
			{
				leds.W(true);
				if (PrevXferComplete)
				{
					RHIDCheckState();
				}
			}
			else
			{
				leds.W(false);
			}
		}
		else
		{
			leds.R(true);
		}




	}


	/*RadioLink channel(&spi, &leds);
	Commander commander(&channel);
	ADCPort adc1(GPIOB, GPIO_Pin_0, ADC_Channel_8);
	//ADCPort adc2(GPIOB, GPIO_Pin_1, ADC_Channel_9);
	SensorProcessor sensorProcessor(&adc1, &adc1);
	Stabilizer stabilizer(&sensorProcessor, &commander);
	Motors motor;
	Controller controller(&stabilizer, &motor);*/
/*
	while(true){
		leds.RGBW(true, true, true, true);
		Delay(1000);
		leds.Off();
		leds.R(true);
		Delay(1000);
		leds.Off();
		leds.G(true);
		Delay(1000);
		leds.Off();
		leds.B(true);
		Delay(1000);
		leds.Off();
		leds.W(true);
		Delay(1000);
		//controller.Update();
		//motor.SetRatio(adc1.Read(), adc1.Read(), adc2.Read(), adc2.Read());
	}
*/
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


