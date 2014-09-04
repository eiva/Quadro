#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_sdio.h"
#include "system_stm32f4xx.h"
#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"

/*******************************************************************/
void vFreeRTOSInitAll()
{
	GPIO_InitTypeDef port;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /////////////////////////////////////////////////////
    // LEDS
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_OUT;
    port.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    port.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &port);

    /////////////////////////////////////////////////////
    // PWM
    // GPIO
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_OType = GPIO_OType_PP;
    port.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    port.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOE,&port);

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);

    // Timer configuration
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 2000; // Total period = 20ms;
    TIM_TimeBaseStructure.TIM_Prescaler =  SystemCoreClock/1000000; // 1MHz devider - timer ticks with 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Enable the timer
    TIM_Cmd(TIM1, ENABLE);
    // Enable the timer PWM outputs
    TIM_CtrlPWMOutputs(TIM1, ENABLE);


    TIM_SetCompare1(TIM1, 25);
    TIM_SetCompare2(TIM1, 75);
    TIM_SetCompare3(TIM1, 120);
    TIM_SetCompare4(TIM1, 250);

    /////////////////////////////////////////////////////
    // SD card initialization
    // GPIO
}

/*******************************************************************/
void vLedTask0 (void *pvParameters)
{
	uint32_t i = 100, delta = 10, step = 10;
    while(1)
    {
    	i += delta;
    	TIM_SetCompare1(TIM1, i);
    	if (i > 2000) delta = -step;
    	if (i < 10 ) delta = +step;
		vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

void vLedTask1 (void *pvParameters)
{
	uint32_t state;
    while(1)
    {
		if (state == 0)
		{
			GPIO_SetBits(GPIOD,GPIO_Pin_13);
			state = 1;
		}
		else
		{
			GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			state = 0;
		}
		vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

void vLedTask2 (void *pvParameters)
{
	uint32_t state, i;
    while(1)
    {
		if (state == 0)
		{
			GPIO_SetBits(GPIOD,GPIO_Pin_14);
			state = 1;
		}
		else
		{
			GPIO_ResetBits(GPIOD,GPIO_Pin_14);
			state = 0;
		}
		vTaskDelay(5);
    }
    vTaskDelete(NULL);
}

void vLedTask3 (void *pvParameters)
{
	uint32_t state, i;
    while(1)
    {
    	taskENTER_CRITICAL();
		if (state == 0)
		{
			GPIO_SetBits(GPIOD,GPIO_Pin_15);
			state = 1;
		}
		else
		{
			GPIO_ResetBits(GPIOD,GPIO_Pin_15);
			state = 0;
		}
		taskEXIT_CRITICAL();
		taskYIELD();
    }
    vTaskDelete(NULL);
}

int main_disabled(void)
{
    vFreeRTOSInitAll();
    xTaskCreate(vLedTask0,(signed char*)"LedTask0", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLedTask1,(signed char*)"LedTask1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLedTask2,(signed char*)"LedTask2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLedTask3,(signed char*)"LedTask3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
}

/*******************************************************************/
void vApplicationIdleHook( void )
{
}



/*******************************************************************/
void vApplicationMallocFailedHook( void )
{
    for( ;; );
}



/*******************************************************************/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    for( ;; );
}



/*******************************************************************/
void vApplicationTickHook( void )
{
}
