#include "hal.h"
#include "io.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SERIAL_USE_DMA

#define MAX_NUM_UARTS 5

#ifdef SERIAL_USE_DMA

#define SERIAL_DMA_BUF_LEN 64
volatile static uint8_t tx_flag = 0;
volatile static uint8_t rx_flag = 0;
SemaphoreHandle_t rx_sem = NULL;

uint8_t serial_tx_buf[SERIAL_DMA_BUF_LEN];
uint8_t serial_rx_buf[SERIAL_DMA_BUF_LEN];
#endif

USART_TypeDef * serial_port[MAX_NUM_UARTS] = {
	USART1,
	USART2,
	USART3,
	UART4,
	UART5
};

struct io_map uart_io_map[] = {
	/* usart 1 */
	{
		"usart1_tx", GPIO_Pin_9, GPIO_PinSource9, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, \
		GPIO_AF_USART1, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AF
	},
	{
		"usart1_rx", GPIO_Pin_10, GPIO_PinSource10, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, \
		GPIO_AF_USART1, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AF
	},
	/* usart 2 */
	{
		"usart2_tx", GPIO_Pin_2, GPIO_PinSource2, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, \
		GPIO_AF_USART2, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},
	{
		"usart2_rx", GPIO_Pin_3, GPIO_PinSource3, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, \
		GPIO_AF_USART2, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},
	/* usart 3 */
	{
		"usart3_tx", GPIO_Pin_2, GPIO_PinSource2, GPIOD, AHB1, RCC_AHB1Periph_GPIOD, \
		GPIO_AF_USART3, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},
	{
		"usart3_rx", GPIO_Pin_3, GPIO_PinSource3, GPIOD, AHB1, RCC_AHB1Periph_GPIOD, \
		GPIO_AF_USART3, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},

	/* usart 4 */
	{
		"usart4_tx", GPIO_Pin_0, GPIO_PinSource0, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, \
		GPIO_AF_UART4, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},
	{
		"usart4_rx", GPIO_Pin_1, GPIO_PinSource1, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, \
		GPIO_AF_UART4, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},

	/* usart 5 */
	{
		"usart5_tx", GPIO_Pin_12, GPIO_PinSource12, GPIOC, AHB1, RCC_AHB1Periph_GPIOC, \
		GPIO_AF_UART5, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	},
	{
		"usart5_rx", GPIO_Pin_2, GPIO_PinSource2, GPIOD, AHB1, RCC_AHB1Periph_GPIOD, \
		GPIO_AF_UART5, GPIO_PuPd_UP, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_Mode_AF
	}
};

int serial_io_init(bus_t bus)
{
	int minor = bus & 0x0f;

	io_request(&uart_io_map[minor]);
	io_request(&uart_io_map[minor+1]);

	switch(minor) {
		case 0:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			break;
		case 1:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			break;
		case 2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			break;
		case 3:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
			break;
		case 4:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
			break;
		default:
			return -1;
	}

	return 0;
}

int serial_enable(bus_t bus, void *priv)
{
	int baudrate = (int)priv;
	int minor = bus & 0x0f;
	USART_InitTypeDef UartStructure;

	USART_OverSampling8Cmd(serial_port[minor], ENABLE);
	USART_StructInit(&UartStructure);
	UartStructure.USART_BaudRate = baudrate;
	USART_Init(serial_port[minor], &UartStructure);

	USART_Cmd(serial_port[minor], ENABLE);
}

#ifdef SERIAL_USE_DMA
int serial_write(uint8_t minor, uint8_t *buf, uint32_t len)
{
	uint8_t offset, size, i;

	offset = 0;
	while (len > 0) {
		if (len <= SERIAL_DMA_BUF_LEN)
			size = len;
		else
			size = SERIAL_DMA_BUF_LEN;

		while(tx_flag) ;
		tx_flag = 1;

		memcpy(serial_tx_buf, buf+offset, size);
		if (minor == 0) {
			DMA_SetCurrDataCounter(DMA2_Stream7, size);
			DMA_Cmd(DMA2_Stream7, ENABLE);
		} else if (minor == 2) {
			DMA_SetCurrDataCounter(DMA1_Stream3, size);
			DMA_Cmd(DMA1_Stream3, ENABLE);
		}

		offset += size;
		len -= size;
	}

	return 0;
}

int serial_read(uint8_t minor, uint8_t *buf, uint32_t len)
{
	uint32_t i;

	for (i=0; i<len; i++) {
		xSemaphoreTake(rx_sem, portMAX_DELAY);
	//	while (!rx_flag) ;
		buf[i] = USART_ReceiveData(serial_port[minor]);
//		rx_flag = 0;
//		printk("recv: 0x%02x", buf[i]);
	}

	return 0;
}
#else
int serial_write(uint8_t minor, uint8_t *buf, uint32_t len)
{
	uint32_t i;

	for (i=0; i<len; i++) {
		USART_SendData(serial_port[minor], buf[i]);
		while (USART_GetFlagStatus(serial_port[minor], USART_FLAG_TC) != SET) ;
	}

	return 0;
}

int serial_read(uint8_t minor, uint8_t *buf, uint32_t len)
{
	uint32_t i, timeout;

	for (i=0; i<len; i++) {
		timeout = 0xffffffff;
		while (USART_GetFlagStatus(serial_port[minor], USART_FLAG_RXNE) == RESET) {
			if (--timeout == 0)
				break;
		};
		buf[i] = USART_ReceiveData(serial_port[minor]);
	}

	return 0;
}
#endif

#ifdef SERIAL_USE_DMA
int serial_request_dma(bus_t bus)
{
	int minor = bus & 0xf;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	switch (minor) {
		case 0:
			NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(serial_port[minor]->DR));
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)serial_tx_buf;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_BufferSize = SERIAL_DMA_BUF_LEN;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream7, &DMA_InitStructure);
			DMA_ITConfig(DMA2_Stream7, DMA_IT_TC,ENABLE);
			break;
		case 3:
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(serial_port[minor]->DR));
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)serial_tx_buf;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_BufferSize = SERIAL_DMA_BUF_LEN;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream3, &DMA_InitStructure);
			DMA_ITConfig(DMA1_Stream3, DMA_IT_TC,ENABLE);              //DMA enable
			break;
		default:
			printk("dma not supported for port %d\n", minor);
			return -1;
	}
		
	USART_DMACmd(serial_port[minor], USART_DMAReq_Tx, ENABLE);
	return 0;
}

void DMA1_Stream3_IRQHandler(void)  
{  
	if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) {
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);  
		DMA_Cmd(DMA1_Stream3, DISABLE);
		tx_flag = 0;
	}
}

void DMA2_Stream7_IRQHandler(void)  
{  
	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)) {
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);  
		DMA_Cmd(DMA2_Stream7, DISABLE);
		tx_flag = 0;
	}
}

void USART1_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
		STM_EVAL_LEDToggle(LED4);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		xSemaphoreGiveFromISR(rx_sem, &lHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);

		//rx_flag = 1;
	}
}

void USART2_IRQHandler(void)
{
}
#endif

int serial_request_irq(bus_t bus)
{
	int minor = bus & 0xf;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	vSemaphoreCreateBinary(rx_sem);
	USART_ITConfig(serial_port[minor], USART_IT_RXNE, ENABLE);
}

int serial_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir)
{
	int minor = bus & 0xf;
	if (dir == IN)
		return serial_read(minor, buf, len);
	else
		return serial_write(minor, buf, len);
}

struct hal_bus uart1 = {
	.name = "uart1",
	.use_dma = true,
	.bus = BUS(UART, 0),
	.priv = (int *)115200,
	.io_init = serial_io_init,
	.request_dma = serial_request_dma,
	.request_irq = serial_request_irq,
	.bus_enable = serial_enable,
	.bus_cfg = NULL,
	.xfer = serial_bus_xfer,
};

int serial_bus_init()
{
	hal_bus_register(&uart1);
	return 0;
}

hal_driver_init(serial_bus_init);
