#include "usart.h"
#include "shell.h"
#include "stm32f1xx_hal.h"

#include <string.h>

#define RX_MAX 256
#define TX_MAX 256

// Receive
static char rx_data[RX_MAX];
static int rx_idx = 0;

// Transmit
static char tx_data[TX_MAX];
static int tx_wptr = 0;
static int tx_rptr = 0;
static int bytesEnqueued = 0;

void processRx(char data) {
	// If we get a newline, first send a carriage return
	if (data == CARRIAGE_RETURN) {

		// Process command
		rx_data[rx_idx] = NULL_CHAR; // Null-terminate RX data
		if (rx_idx) {
			add_cmd(rx_data, strlen(rx_data) + 1); // Account for null terminator
		} else {
			add_cmd("NEWLINE", 8); // Account for null terminator
		}

		rx_idx = 0;
	}
	else if (data == DELETE) {
		if (rx_idx) {
			// Delete previous char, if we have any -- do not delete prompt
			sendData(BACKSPACE, 3);
			rx_data[--rx_idx] = NULL_CHAR;
		}
	} else {
		rx_data[rx_idx++] = data;
		rx_idx %= RX_MAX;
		USART2->DR = data;
	}


}

void sendData(const char* data, int len){
	// Make sure length can fit in send buffer
	int lenToSend = len > TX_MAX ? TX_MAX : len;

	// If we're running out of room, pause to send some
	int bufRemaining = TX_MAX - bytesEnqueued;
	if (lenToSend > bufRemaining) {
		for (int i = 0; i < bytesEnqueued; i++) {
			writeUartByte(1);
		}
	}

	for (int i = 0; i < lenToSend; i++) {
		tx_data[tx_wptr++] = data[i];
		tx_wptr %= TX_MAX;
	}
	bytesEnqueued += lenToSend;
	USART2->CR1 |= (1 << 7); // Set TXEIE
}

void writeUartByte(int blocking) {
	if (!(USART2->SR & UART_FLAG_TXE)) {
		if (!blocking) {
			return;
		}

		while(!(USART2->SR & UART_FLAG_TXE)) {} // block for flag
	}

	if (!bytesEnqueued) {
		USART2->CR1 &= ~(1 << 7); // Clear TXEIE
		return;
	}

	USART2->DR = tx_data[tx_rptr++];
	tx_rptr %= TX_MAX;
	bytesEnqueued--;
}

int tx_above_watermark() {
	return bytesEnqueued >= (TX_MAX / 2);
}
