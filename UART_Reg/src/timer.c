#include "timer.h"
#include "stm32f103xb.h"

static int timer_done = 0;

void start_countdown_blocking(TimerInit_t* timer_init) {
	// Enable (stop) the timer in case it's running
	TIM2->CR1 = 0;

	// Set the prescaler to dial down the frequency fed to TIMx_CNT
	// Subtract 1 because prescaler  goes through (PSC+1) ticks
	TIM2->PSC = (timer_init->prescaler - 1);

	// Set the auto reload register with our countdown
	// Subtract 1 because timer goes through (ARR+1) ticks
	TIM2->ARR = (timer_init->count - 1);

	// Counting direction
	if (timer_init->direction == COUNT_DOWN) {
		TIM2->CR1 |= (0b1 << 4);
	}

	// One-shot
	if (timer_init->pulse_mode == ONE_SHOT) {
		TIM2->CR1 |= (0b1 << 3);
	}

	// Causes prescaler load (+ CNT load with ARR, for count-down mode)
	TIM2->EGR = TIM_EGR_UG;

	// Clear UIF from software update event
	TIM2->SR = 0;

	// Enable (start) the timer
	TIM2->CR1 |= (0b1 << 0);

	// wait for flag
	while(0 == (TIM2->SR & 1)) {
		// Wait
	}
	TIM2->SR = 0;
	timer_done++;

}
