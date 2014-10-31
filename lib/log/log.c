#include "log.h"
#include "hal.h"

#define PRINT_TO_USART
#define PRINT_PORT 0
#define BAUDRATE 115200

#define MAX_BUF_SIZE 1024

int printk(const char * format, ...)
{
	int count;
	char buffer[MAX_BUF_SIZE];
	va_list args;
	va_start(args, format);
#ifdef PRINT_TO_USART
	count = vsnprintf((char *)buffer, MAX_BUF_SIZE-1, format, args);
	if (buffer[count-1] == '\n') {
		buffer[count-1] == '\n';
		buffer[count] == '\r';
		buffer[count+1] == '0';
		count++;
	}

	hal_bus_xfer(BUS(UART, 0), 0, buffer, count, OUT);
//	serial_write(PRINT_PORT, uart_buffer, numBytes);
#else
	count = vprintf(format, args);
#endif
	va_end(args);
	return count;
}

#if 0
int printk(const char * format, ...)
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
#endif

/*
void serial_read_line(uint8_t port_num, char *str)
{
	uint8_t c;

	do {
		while (USART_GetFlagStatus(serial_port[port_num], USART_FLAG_RXNE) == RESET) ;
		c = USART_ReceiveData(serial_port[port_num]);
		printk("%c", c);
		*str = c;
		str++;
	} while (c != '\r' && c != '\n');

	*str = '\0';
}
*/
