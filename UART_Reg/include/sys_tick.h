#ifndef _SYS_TICK_H_
#define _SYS_TICK_H_

#include "stm32f103xb.h"
#include <stdint.h>

// Setup and configuration
void init_sys_tick(uint32_t reload_val);

// System time / ticks
void tick();
uint32_t get_ticks();

#endif // _SYS_TICK_H_
