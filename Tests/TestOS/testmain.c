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


int main(void)
{
	SystemInit();
	GPIO_InitTypeDef port;
	FATFS Fatfs[_VOLUMES];
	FIL File;

	uint index;

	uint8_t buffer[512];
	for (index = 0; index < 512; ++index)
	{
		buffer[index] = index;
	}
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &port);



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

	//Res = SD_Init();

    //Res = SD_WriteBlock(buffer, 0, 512);

	  // Read operation as described in Section B
	//Res = SD_ReadBlock(buffer, 0, 512);


	//Res = disk_initialize(0);
	FRes = f_mount(&Fatfs, "", 0);
	//FRes = f_mkfs("", 1, 512);
	FRes = f_open(&File,"output.txt", FA_WRITE | FA_OPEN_ALWAYS);
	FRes = f_write(&File, buffer, 512, &index);
	FRes = f_close(&File);
	while(1);
	return 0;
}


