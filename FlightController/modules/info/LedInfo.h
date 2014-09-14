#pragma once
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

// Led information display
class LedInfo{

public:
  LedInfo()
	  {
	  	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  	  GPIO_InitTypeDef port;
	  	  GPIO_StructInit(&port);
	  	  port.GPIO_Mode = GPIO_Mode_OUT;
	      port.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	      port.GPIO_Speed = GPIO_Speed_2MHz;
	      GPIO_Init(GPIOD, &port);
	  }

  void R(bool on){ // 14
	  GPIO_WriteBit(GPIOD, GPIO_Pin_14, on?Bit_SET:Bit_RESET);
  }
  void G(bool on){ // 12
	  GPIO_WriteBit(GPIOD, GPIO_Pin_12, on?Bit_SET:Bit_RESET);
  }
  void B(bool on){ // 15
	  GPIO_WriteBit(GPIOD, GPIO_Pin_15, on?Bit_SET:Bit_RESET);
  }
  void Y(bool on){ //13
	  GPIO_WriteBit(GPIOD, GPIO_Pin_13, on?Bit_SET:Bit_RESET);
  }
  void RGBY(bool r, bool g, bool b, bool y){
    R(r); G(g); B(b); Y(y);
  }
  void Off(){
	  GPIO_WriteBit(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, Bit_RESET);
  }
};
