#include "my_uart.h"

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  UART_SendByte(USART1, (uint8_t) ch);
  return ch;
}

extern UART_HandleTypeDef huart1;

void UART_Init(void)
{
	__HAL_UART_ENABLE(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}
uint8_t UART_WaitUntilTxEmpty(USART_TypeDef *USARTx)
{
	int timeOut = 0;
	while((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) ? SET : RESET) == RESET )
	{
		timeOut++;
		if(timeOut > 10000) return 0;
	}
	return 1;
}

void UART_SendByte(USART_TypeDef *USARTx, uint8_t data)
{
	UART_WaitUntilTxEmpty(USARTx);
	USARTx->DR = data;
}


void UART_SendString(USART_TypeDef* USARTx, char *Str)
{
	while(*Str)
	{
		UART_SendByte(USARTx,*Str); 
		Str++;
	}
}

void UART_SendData(USART_TypeDef* USARTx, uint8_t *data, uint8_t length)
{
	for(uint8_t i=0; i< length; i++)
	{
		UART_SendByte(USARTx,data[i]); 
	}
}

__weak void UART_ReceiveFunctionCallBack(uint8_t data)
{
}

