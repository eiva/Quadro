#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_exti.h>
#include <misc.h>
#include <semihosting.h>
#include <stdio.h>

#include"RF/nRF24L01.h"

#define RX_PAYLOAD           1

uint8_t buf[RX_PAYLOAD];
uint8_t have_data;

void EXTI15_10_IRQHandler(void) {
	uint8_t i;
	uint8_t status;
	GPIOC->ODR ^= GPIO_Pin_9; // Toggle LED
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
/*		for (i = 0; i < RX_PAYLOAD; i++) buf[i] = 0x00;
		status = nRF24_RXPacket(buf,RX_PAYLOAD);
		nRF24_ClearIRQFlags();

		if (status == 0x0E) {
//			UART_SendStr(" => FIFO Empty (fake alarm)\n");
		} else {
			have_data = 1;
//			UART_SendChar('\n');
		}

		nRF24_RXMode(RX_PAYLOAD);
		nRF24_ClearIRQFlags();
*/

	}
	EXTI_ClearITPendingBit(EXTI_Line10);
}

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*EXTI_InitTypeDef EXTIInit;
	EXTI_StructInit(&EXTIInit);
	EXTIInit.EXTI_Line = EXTI_Line10;             // EXTI will be on line 10
	EXTIInit.EXTI_LineCmd = ENABLE;               // EXTI1 enabled
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;     // Generate IRQ
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling; // IRQ on signal falling
	EXTI_Init(&EXTIInit);

	// Configure EXTI1 interrupt
	NVIC_InitTypeDef NVICInit;
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVICInit.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_Init(&NVICInit);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
	*/
	GPIO_InitTypeDef PORT;
	GPIO_StructInit(&PORT);
	PORT.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	nRF24_init();

	if (nRF24_Check() != 0) {
		GPIOB->ODR ^= GPIO_Pin_15; // Toggle LED
		GPIOB->ODR ^= GPIO_Pin_14; // Toggle LED
		while(1);
	}
	while(1);
	nRF24_RXMode(RX_PAYLOAD);
	nRF24_ClearIRQFlags();
	unsigned char i,v;

		uint8_t status;
		while(1){
			if (nRF24_DataReady())
			{
				for (i = 0; i < RX_PAYLOAD; i++) buf[i] = 0x00;
				status = nRF24_RXPacket(buf,RX_PAYLOAD);

				if (status == 0x0E) {
				//			UART_SendStr(" => FIFO Empty (fake alarm)\n");
				} else {
				have_data = 1;
				//			UART_SendChar('\n');
					GPIOC->ODR ^= GPIO_Pin_9; // Toggle LED
					int recievedData = buf[0];
					//printf("Tick:%d\n",recievedData);
				}
				nRF24_RXMode(RX_PAYLOAD);
				nRF24_ClearIRQFlags();
			} else {
			GPIOC->ODR ^= GPIO_Pin_8; // Toggle LED
			}
		}
}
