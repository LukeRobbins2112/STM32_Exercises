#include "shell.h"
#include "usart.h"
#include "string.h"
#include <stdint.h>
#include <stdio.h>

#define MAX_CMD_LEN 32
#define QUEUE_LENGTH 1
#define NUM_COMMANDS 16 //@TODO how to dynamically add via macro? See Merlot

// Shell Command Lookup Table
typedef int (*CmdFunc)(void);
struct Command {
	char cmd[MAX_CMD_LEN];
	CmdFunc func;
};
static struct Command cmd_table[NUM_COMMANDS];


// Queue of commands to process -- from outside ISR
struct ShellCmd {
	char cmd[MAX_CMD_LEN];
	char is_available;
};
static struct ShellCmd shell_queue[QUEUE_LENGTH];
static int queue_idx = 0;

void initialize_shell() {
	// Initialize processing queue
	for (int i = 0; i < QUEUE_LENGTH; i++) {
		shell_queue[i].is_available = 1;
	}

	// Commands
	cmd_table[0] = (struct Command){.cmd = "NEWLINE", .func = print_newline};
	cmd_table[1] = (struct Command){.cmd = "UNKNOWN", .func = unknown_cmd};
	cmd_table[2] = (struct Command){.cmd = "GPIOA", .func = dumpGPIOA};
}

void add_cmd(const char* cmd, int cmd_len) {
	if (strlen(cmd) >= MAX_CMD_LEN) {
		// Cannot be longer than max length, ignore
		// >= because we need one char for null terminator
		return;
	}

	int ptr = queue_idx;
	while(!shell_queue[ptr].is_available) {
		ptr++;
		ptr %= QUEUE_LENGTH;
	}

	struct ShellCmd* cmd_ptr = &(shell_queue[ptr]);

	memcpy(cmd_ptr->cmd, cmd, cmd_len);
	cmd_ptr->cmd[cmd_len] = '\0'; // Insurance
	cmd_ptr->is_available = 0;

	++queue_idx;
	queue_idx %= QUEUE_LENGTH;
}

void process_next_cmd() {
	struct ShellCmd* cmd;
	for (int i = 0; i < QUEUE_LENGTH; i++) {
		if (!shell_queue[i].is_available) {
			cmd = &(shell_queue[i]);
			break;
		}
	}

	if (!cmd) {
		return;
	}

	// Use lookup table of handler functions, based on provided command name
	// @TODO handle multiple words, separated by spaces -- for arguments
	int cmd_found = 0;
	for (int i = 0; i < NUM_COMMANDS; i++) {
		if (strcmp(cmd->cmd, cmd_table[i].cmd) == 0) {
			cmd_found = 1;
			cmd_table[i].func();
		}
	}

	if (!cmd_found) {
		unknown_cmd();
	}

	// Valid or invalid, we processed the command
	cmd->is_available = 1; // Mark as processed
}

// Assumes data is shifted into bottom 4 bits
static void add_port_cfg(char* buf, uint8_t regVal) {
	char is_input = 0;
	// Add input/output
	uint8_t mode_flags = regVal & 0x3;
	switch (mode_flags) {
		case 0x0:
			strcat(buf, "IN_");
			is_input = 1;
			break;
		case 0x1:
			strcat(buf, "OUT10_");
			break;
		case 0x2:
			strcat(buf, "OUT2_");
			break;
		case 0x3:
			strcat(buf, "OUT50_");
			break;
	}

	uint8_t cfg_flags = (regVal >> 2) & 0x3;
	switch (cfg_flags) {
		case 0x0:
			strcat(buf, is_input ? "Anlg" : "GP-PP");
			break;
		case 0x1:
			strcat(buf, is_input ? "Flt" : "GP-OD");
			break;
		case 0x2:
			strcat(buf, is_input ? "Pull" : "AF-PP");
			break;
		case 0x3:
			strcat(buf, is_input ? "Res" : "AF-OD");
			break;
	}

	strcat(buf, " ");
}

int dumpGPIOA() {
	GPIO_TypeDef* gpio;
	gpio = GPIOA;
	sendData("\r\nGPIOA:\r\n", 10);

	char buf[32] = ""; // Initialize so there is null terminator for strcpy

//	if (strcmp(gpio_name, "GPIOA") == 0) {
//		gpio = GPIOA;
//		sendData("\r\nGPIOA:\r\n", 10);
//	} else if (strcmp(gpio_name, "GPIOB") == 0) {
//		gpio = GPIOB;
//		sendData("GPIOB\r\n", 7);
//	} else if (strcmp(gpio_name, "GPIOC") == 0) {
//		gpio = GPIOC;
//		sendData("GPIOC\r\n", 7);
//	} else if (strcmp(gpio_name, "GPIOD") == 0) {
//		gpio = GPIOD;
//		sendData("GPIOD\r\n", 7);
//	} else {
//		// Unknown (for now)
//		sendData("Bad GPIO\r\n", 10);
//		return;
//	}


	for (int i = 0; i < 16; i++) {
		if (i && i%4 == 0) {
			sendData("\r\n", 2);
		}
		sprintf(buf, "Pin%d: ", i);

		// Get CRL or CRL
		uint32_t gpio_reg = (i < 8) ? (gpio->CRL) : (gpio->CRH);
		uint8_t regByte = gpio_reg >> ((i%8) * 4);

		add_port_cfg(buf, regByte);
		sendData(buf, strlen(buf));
	}

	return 0;
}

int print_newline() {
	sendData(PROMPT, strlen(PROMPT));
	return 0;
}

int unknown_cmd() {
	const char* msg = "\r\nUNKNOWN COMMAND\r\n";
	sendData(msg, strlen(msg));
	print_newline();
	return 0;
}
