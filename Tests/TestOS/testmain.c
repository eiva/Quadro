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

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

//******************************************************************************

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/******************************************************************************/
/**
  * @brief  This function handles SDIO global interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

/**
  * @brief  This function handles DMA2 Stream3 or DMA2 Stream6 global interrupts
  *         requests.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)//SD_SDIO_DMA_IRQHANDLER
{
  /* Process DMA2 Stream3 or DMA2 Stream6 Interrupt Sources */
  SD_ProcessDMAIRQ();
}

int main(void)
{
	SystemInit();
	GPIO_InitTypeDef port;
	FATFS Fatfs[_VOLUMES];
	FIL File;

	uint count_rw_ok;

	uint8_t buffer[512];
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

	Res = SD_Init();
	Res = disk_initialize(0);
	FRes = f_mount(&Fatfs, "", 1);
	FRes = f_open(&File,"output.txt", FA_WRITE | FA_CREATE_NEW);
	FRes = f_write(&File, buffer, 512, &count_rw_ok);
	FRes = f_close(&File);

	return 0;
}


