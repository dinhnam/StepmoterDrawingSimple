#ifndef MY_UART_H
#define MY_UART_H
#include "stm32f1xx_hal.h"
#include <stdio.h>

void UART_Init(void);
uint8_t UART_WaitUntilTxEmpty(USART_TypeDef *uart);
void 	UART_SendByte(USART_TypeDef* USARTx, uint8_t Data);
void UART_SendString(USART_TypeDef* USARTx, char *Str);
void UART_SendData(USART_TypeDef* USARTx, uint8_t *data, uint8_t length);
void UART_ReceiveFunctionCallBack(uint8_t data);
#endif
