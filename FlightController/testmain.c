#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_sdio.h"
#include "system_stm32f4xx.h"

#include "sdio_sd.h"
#include "ff_support.h"
#include "ff.h"


int mainTestSDIO(void)
{
	SystemInit();
	GPIO_InitTypeDef port;
	FATFS Fatfs[_VOLUMES];
	FIL File;

	uint index;

	uint8_t buffer[512*2];
	for (index = 0; index < 512*2; ++index)
	{
		buffer[index] = index;
	}
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &port);

    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_IN;
    port.GPIO_Pin = GPIO_Pin_1;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    port.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &port);

    GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    while(1){
    	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == RESET)
    	{
    		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == RESET)
    		{
    			break;
    		}
    	}
    }
    GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

	SD_Error Res;
	FRESULT FRes;
	  //NVIC_Configuration();


	  /*//SD_ReadBlock(buffer,0, 512);
	  //SD_WaitReadOperation();
	  //while(SD_GetStatus() != SD_TRANSFER_OK);
	  Res = disk_initialize(0);
	  FRes = f_mount(0, &Fatfs, 1);

	  FRes = f_open(&File,"output.txt", FA_WRITE);
	  FRes = f_write(&File, buffer, 512, &count_rw_ok);
	  FRes = f_close(&File);

	  SD_DeInit();
*/
	//GPIO_SetBits(GPIOD, GPIO_Pin_13);
	/*Res = SD_Init();

	for (index = 0; index < 512; ++index)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		Res = SD_WriteBlock(buffer, index*512, 512);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		if (Res != SD_OK)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
		    while(1);
		}
	}*/

    //GPIO_SetBits(GPIOD, GPIO_Pin_13);

    //SD_DeInit();

    // Read operation as described in Section B


    //GPIO_SetBits(GPIOD, GPIO_Pin_14);
	//Res = disk_initialize(0);

	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	FRes = f_mount(&Fatfs, "", 0);
	if (FRes != RES_OK) while(1);

	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	FRes = f_open(&File,"output.txt", FA_WRITE | FA_OPEN_ALWAYS);
	if (FRes != RES_OK) while(1);

	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	FRes = f_write(&File, buffer, 512*2, &index);
	if (FRes != RES_OK) while(1);

	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	FRes = f_close(&File);
	if (FRes != RES_OK) while(1);

	GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	f_mount(NULL, "", 0);

	while(1);
	return 0;
}


