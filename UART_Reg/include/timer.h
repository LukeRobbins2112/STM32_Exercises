#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>

typedef enum CountDirection {
	COUNT_UP = 0,
	COUNT_DOWN = 1,
} CountDirection_e;

typedef enum PulseMode {
	CONTINUOUS = 0,
	ONE_SHOT = 1,
} PulseMode_e;

typedef enum TimerAction {
	BLOCK = 0,
	INTERRUPT = 1,
} TimerAction_e;

typedef struct TimerInit {
	uint16_t count;
	uint16_t prescaler;
	CountDirection_e direction;
	PulseMode_e pulse_mode;
	TimerAction_e timer_action;
} TimerInit_t;

void init_timer(TimerInit_t* timer_init);

void timer_fired();

#endif // _TIMER_H_
