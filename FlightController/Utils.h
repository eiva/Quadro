#ifdef __cplusplus
 extern "C" {
#endif

static __IO uint32_t TimingDelay;
 //Функция временной задержки
void Delay(__IO uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

 extern "C" void SysTick_Handler(void){
 	if (TimingDelay != 0x00)
 		TimingDelay--;
 }

#ifdef __cplusplus
}
#endif
