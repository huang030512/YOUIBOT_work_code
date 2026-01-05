#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "usart.h"
#include "string.h"
//ÖØ¶¨Ïò
void USART_printf(char *fmt, ...)
{
	va_list ap;
	char str[128];
	va_start(ap,fmt);
	vsprintf(str,fmt,ap);
	va_end(ap);
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str),0xFFFF);
}



