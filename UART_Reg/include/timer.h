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

typedef struct TimerInit {
	uint16_t count;
	uint16_t prescaler;
	CountDirection_e direction;
	PulseMode_e pulse_mode;
} TimerInit_t;

void start_countdown_blocking(TimerInit_t* timer_init);

#endif // _TIMER_H_
