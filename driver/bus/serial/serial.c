#include "serial.h"
#include "stm32f4_discovery.h"
#include <stdio.h>
#include <stdarg.h>

#define MAX_NUM_UARTS 5

static USART_TypeDef* UARTS[MAX_NUM_UARTS] = {
	USART1,
	USART2,
	USART3,
	UART4,
	UART5
};

void serial_init(uint32_t port, uint32_t baudrate)
{
	USART_InitTypeDef UartStructure;
	USART_ClockInitTypeDef UartClockStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_TypeDef *usart;

	if (port > 4) return ;

	switch(port) {
		case 0:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

			GPIO_StructInit(&GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
			break;
		case 1:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

			GPIO_StructInit(&GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
			break;
		case 2:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

			GPIO_StructInit(&GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOD, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
			break;
		case 3:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
			break;
		case 4:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

			GPIO_StructInit(&GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOC, &GPIO_InitStructure);

			GPIO_StructInit(&GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOD, &GPIO_InitStructure);

			/*
			myNVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
			myNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
			myNVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
			myNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&myNVIC_InitStructure); 
			*/

			GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

			/*
			UartClockStructure.USART_Clock = USART_Clock_Disable;
			UartClockStructure.USART_CPOL = USART_CPOL_Low;
			UartClockStructure.USART_CPHA = USART_CPHA_2Edge;
			UartClockStructure.USART_LastBit = USART_LastBit_Disable;
			USART_ClockInit(UART5, &UartClockStructure);
			*/

			//USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
			break;
		default:
			printf("invalid usart number\n");
	}

	USART_StructInit(&UartStructure);
	UartStructure.USART_BaudRate = baudrate;
	USART_Init(UARTS[port], &UartStructure);

	USART_Cmd(UARTS[port], ENABLE);
}

void serial_write(uint32_t port, uint8_t *buf, uint32_t len)
{
	uint32_t i;

	for (i=0; i<len; i++) {
		USART_SendData(UARTS[port], buf[i]);
		while (USART_GetFlagStatus(UARTS[port], USART_FLAG_TXE) == RESET) ;
	}
}

void serial_read(uint32_t port, uint8_t *buf, uint32_t len)
{
	uint32_t i, timeout;

	for (i=0; i<len; i++) {
		timeout = 0xffffffff;
		while (USART_GetFlagStatus(UARTS[port], USART_FLAG_RXNE) == RESET) {
			if (--timeout == 0)
				break;
		};
		buf[i] = USART_ReceiveData(UARTS[port]);
	}
}

void serial_read_line(uint32_t port, char *str)
{
	uint8_t c;

	do {
		while (USART_GetFlagStatus(UARTS[port], USART_FLAG_RXNE) == RESET) ;
		c = USART_ReceiveData(UARTS[port]);
		serial_print("%c", c);
		*str = c;
		str++;
	} while (c != '\r' && c != '\n');

	*str = '\0';
}


#define PRINT_TO_USART

int16_t serial_print(const char * format, ...)
{
	int16_t numBytes;
	uint8_t uart_buffer[256];
	va_list args;
	va_start(args, format);
#ifdef PRINT_TO_USART
	numBytes = vsprintf((char *)uart_buffer, format, args);
	serial_write(USART_PORT, uart_buffer, numBytes);
#else
	numBytes = vprintf(format, args);
#endif
	va_end(args);
	return numBytes;
}

int16_t serial_println(const char * format, ...)
{
	int16_t numBytes;
	uint8_t uart_buffer[256];
	va_list args;
	va_start(args, format);
#ifdef PRINT_TO_USART
	numBytes = vsprintf((char *)uart_buffer, format, args);
	serial_write(USART_PORT, uart_buffer, numBytes);
	serial_write(USART_PORT, "\r\n", 2);
#else
	numBytes = vprintf(format, args);
#endif
	va_end(args);
	return numBytes;
}
