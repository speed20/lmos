#include "log.h"

#define PRINT_TO_USART
#define PRINT_PORT 0
#define BAUDRATE 115200

int serial_print(const char * format, ...)
{
	int numBytes;
	char uart_buffer[256];
	va_list args;
	va_start(args, format);
#ifdef PRINT_TO_USART
	numBytes = vsprintf((char *)uart_buffer, format, args);
	serial_write(PRINT_PORT, uart_buffer, numBytes);
#else
	numBytes = vprintf(format, args);
#endif
	va_end(args);
	return numBytes;
}

int serial_println(const char * format, ...)
{
	int numBytes;
	char uart_buffer[256];
	va_list args;
	va_start(args, format);
#ifdef PRINT_TO_USART
	numBytes = vsprintf((char *)uart_buffer, format, args);
	serial_write(PRINT_PORT, uart_buffer, numBytes);
	serial_write(PRINT_PORT, "\r\n", 2);
#else
	numBytes = vprintf(format, args);
#endif
	va_end(args);
	return numBytes;
}

/*
void serial_read_line(uint8_t port_num, char *str)
{
	uint8_t c;

	do {
		while (USART_GetFlagStatus(serial_port[port_num], USART_FLAG_RXNE) == RESET) ;
		c = USART_ReceiveData(serial_port[port_num]);
		serial_print("%c", c);
		*str = c;
		str++;
	} while (c != '\r' && c != '\n');

	*str = '\0';
}
*/
