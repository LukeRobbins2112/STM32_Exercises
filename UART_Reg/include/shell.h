#ifndef _SHELL_H_
#define _SHELL_H_

#include "stm32f1xx_hal.h"

// Shell management
void initialize_shell();
void add_cmd(const char* cmd, int cmd_len);
void process_next_cmd();

// Specific shell commands
int print_newline();
int dumpGPIOA();


#endif // _SHELL_H_
