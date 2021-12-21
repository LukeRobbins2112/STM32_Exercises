#ifndef _USART_H_
#define _USART_H_

#include <stdlib.h>
#include "stm32f103xb.h"

#define BREAK = '\n';
#define PROMPT "\r\n> "

void processRx(char data);
void sendData(const char* data, int len);
void writeUartByte();

#endif // _USART_H_
