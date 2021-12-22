#ifndef _USART_H_
#define _USART_H_

#include <stdlib.h>
#include "stm32f103xb.h"

#define BREAK '\n';
#define DELETE '\177'
#define CARRIAGE_RETURN '\r'
#define NULL_CHAR '\0'
#define BACKSPACE "\b \b"
#define PROMPT "\r\nuart$ "

void processRx(char data);
void sendData(const char* data, int len);
void writeUartByte();

#endif // _USART_H_
