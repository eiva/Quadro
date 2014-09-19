
#include "stm32f4xx_conf.h"
#include "Port.h"
#include "telemetry.h"
/**************************************************************************************/


int main(void)
{
  SystemInit();
  InitMAVLink();
  ProcessMAVLink();
  int i;
  while(true)
  {
	  for(i = 0; i < 1000000; ++i);
	  ProcessMAVLink();
  }
  /*UsartInterface usart(USART1, 57600);
  Port p(GPIOB, GPIO_Pin_9);
  Port l(GPIOB, GPIO_Pin_8);
  p.Low();
  l.Low();
  int i;
  //uint8_t ch = 0;
  while(1){
	  for(i = 0; i < 1000000; ++i);
	  p.High();
	  //while(!(USART1->SR & USART_SR_TC));
	  l.High();
	  //while (USART_GetFlagStatus(USART1, USART_SR_TC) == SET);
	  //USART_SendData(USART1, ch);
	  //ch++;
	  usart.Put(10, (const uint8_t*)"Hello from hell inside STM\r\n");
	  l.Low();
	  //while(!(USART1->SR & USART_SR_TC));
	  //while (USART_GetFlagStatus(USART1, USART_SR_TC) == SET);
	  p.Low();
  }*/
}



