#include "sys_tick.h"

#define MSEC_PER_SEC 1000u

static uint32_t Ticks = 0;

void init_sys_tick(uint32_t reload_val) {
	// Set up STRELOAD with reload val -- only uses lower 24 bits (high 24 are reserved/unused)
	// Subtract 1 because the counter goes down to and including 0 before overflow
	reload_val &= SysTick_LOAD_RELOAD_Msk; // Only use lower 24 bits
	SysTick->LOAD = (reload_val - 1);

	// Enable systick interrupts
	// For now, always use AHB clock
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Use AHB


	// Enable SysTick to start the count down
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void tick() {
	++Ticks;
}

uint32_t get_ticks() {
	return Ticks;
}



// @TODO implement this when you have the energy to calculate frequency from RCC->CFGR settings
//void set_systick_interval_ms(uint32_t period_ms) {
//	uint32_t clock_freq = ???;
//	uint32_t reload_val = (clock_freq / MSEC_PER_SEC) * period_ms; // clock ticks/ms * num_ms
//	reload_val &= SysTick_LOAD_RELOAD_Msk; // Only use lower 24 bits
//	SysTick->LOAD = (reload_val - 1);
//
//}
