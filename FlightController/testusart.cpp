
#include "stm32f4xx_conf.h"
#include "Port.h"
/**************************************************************************************/

void RCC_Configuration(void)
{
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Enable UART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

/**************************************************************************************/

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  // TX RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
}

/**************************************************************************************/

void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 57600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = /*USART_Mode_Rx |*/ USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

void USART_puts(USART_TypeDef* USARTx, char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

int main(void)
{
  SystemInit();

  RCC_Configuration();

  GPIO_Configuration();

  USART_Configuration();

  Port p(GPIOB, GPIO_Pin_9);
  Port l(GPIOB, GPIO_Pin_8);
  p.Low();
  l.Low();
  int i;
  //uint8_t ch = 0;
  while(1){
	  for(i = 0; i < 100000; ++i);
	  p.High();
	  //while(!(USART1->SR & USART_SR_TC));
	  l.High();
	  //while (USART_GetFlagStatus(USART1, USART_SR_TC) == SET);
	  //USART_SendData(USART1, ch);
	  //ch++;
	  USART_puts(USART1, "Hello from hell inside STM\r\n");
	  l.Low();
	  //while(!(USART1->SR & USART_SR_TC));
	  //while (USART_GetFlagStatus(USART1, USART_SR_TC) == SET);
	  p.Low();
  }
}



