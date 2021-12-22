#include "usart.h"
#include "stm32f1xx_hal.h"

#include <string.h>

#define RX_MAX 32
#define TX_MAX 32

// Receive
static char rx_data[RX_MAX];
static int rx_idx = 0;

// Transmit
static char tx_data[TX_MAX];
static int tx_idx = 0;
static int bytesToSend = 0;

void processRx(char data) {
	// If we get a newline, first send a carriage return
	if (data == CARRIAGE_RETURN) {
		sendData(PROMPT, 8);
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

// @TODO handle if data already in buffer
void sendData(const char* data, int len){
	int lenToSend = len > 32 ? TX_MAX : len;
	memcpy(tx_data, data, lenToSend);
	bytesToSend = lenToSend;
	USART2->CR1 |= (1 << 7); // Set TXEIE
}

void writeUartByte() {
	if (!(USART2->SR & UART_FLAG_TXE)) {
		// Not empty -- skip write.
		// @TODO write blocking version
		return;
	}

	if (!bytesToSend) {
		tx_idx = 0; // Reset for next write
		USART2->CR1 &= ~(1 << 7); // Clear TXEIE
		return;
	}

	USART2->DR = tx_data[tx_idx++];
	bytesToSend--;
}
