#include "sys_tick.h"

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
