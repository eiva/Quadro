#include <stm32f10x.h>
#include "Utils.h"

#define CYCLES_PER_MICROSECOND  72
#define SYSTICK_RELOAD_VAL      (CYCLES_PER_MICROSECOND * 1000 - 1) /* takes a cycle to reload */
#define US_PER_MS               1000

// Time delay in millis for current Delay operation
volatile uint32_t TimingDelay;

// Uptime in millis.
volatile uint32_t UptimeMillis;

//Функция временной задержки
void Delay(volatile uint32_t nTime){
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

uint32_t Millis(){
	return UptimeMillis;
}

/**
 * @brief Returns the current value of the SysTick counter.
 */
static inline uint32_t systick_get_count(void) {
    return SysTick->VAL;
}

/**
 * Returns time (in microseconds) since the beginning of program
 * execution.  On overflow, restarts at 0.
 * @see millis()
 */
uint32_t Micros(void) {
    uint32_t ms;
    uint32_t cycle_cnt;

    do {
        ms = UptimeMillis;
        cycle_cnt = systick_get_count();
    } while (ms != millis());


    /* SYSTICK_RELOAD_VAL is 1 less than the number of cycles it
     * actually takes to complete a SysTick reload */
    return ((ms * US_PER_MS) +
            (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND);
}

void SysTick_Handler(void){
	++UptimeMillis;
	if (TimingDelay != 0x00)
		--TimingDelay;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
