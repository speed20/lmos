//#include <stdio.h>
#include "FreeRTOS.h"
#include "hal.h"
#include "semphr.h"
#include "io.h"

SPI_TypeDef *spi_bus[] = {SPI1, SPI2, SPI3};

struct io_map spi2_io_map[] = {
	{
		"spi2_clk", GPIO_Pin_3, GPIO_PinSource3, GPIOD, AHB1, RCC_AHB1Periph_GPIOD, \
		GPIO_AF_SPI2, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AF
	},
	{
		"spi2_mosi", GPIO_Pin_14, GPIO_PinSource14, GPIOB, AHB1, RCC_AHB1Periph_GPIOB, \
		GPIO_AF_SPI2, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AF
	},
	{
		"spi2_miso", GPIO_Pin_15, GPIO_PinSource15, GPIOB, AHB1, RCC_AHB1Periph_GPIOB, \
		GPIO_AF_SPI2, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AF
	}
};

SemaphoreHandle_t spi_tx_sem = NULL;
SemaphoreHandle_t spi_rx_sem = NULL;

int spi_io_init(bus_t bus)
{
	int ret = 0;
	int major = TO_MAJOR(bus);
	int minor = TO_MINOR(bus);

    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	if (major != SPI) {
		return -1;
	}

	switch(minor) {
		case 0:
			{
				printk("spi%d not defined\n", minor+1);
				ret = -1;
				break;
			}
		case 1:
			{
				int i;
				for (i=0; i<3; i++) {
					io_request(&spi2_io_map[i]);
				}
				break;
			}
		case 2:
			{
				printk("spi%d not defined\n", minor+1);
				ret = -1;
				break;
			}
	}

	return ret;
}

int spi_request_dma(bus_t bus)
{
	printk("spi request dma, bus: %d", bus);
}

int spi_bus_enable(bus_t bus, void *arg)
{
    SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_SSOutputCmd(SPI2, ENABLE);
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);

	return 0;
}

int spi_bus_cfg(bus_t bus, void *cfg)
{
}

int spi_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir)
{
	printk("%s %d bytes\n", dir == IN ? "receive" : "send", len);
    uint32_t timeout;
    uint8_t val;
	int minor = TO_MAJOR(bus);
	int i;

	for (i=0; i<len; i++) {
		timeout = 0x1000;
		while (SPI_I2S_GetFlagStatus(spi_bus[minor], SPI_I2S_FLAG_TXE) == RESET) {
			if(timeout-- == 0) {
				printk("spi send timeout\n");
				break;
			}   
		}   

		SPI_I2S_SendData(spi_bus[minor], buf[i]);

		timeout = 0x1000;
		while (SPI_I2S_GetFlagStatus(spi_bus[minor], SPI_I2S_FLAG_RXNE) == RESET) {
			if(timeout-- == 0) {
				printk("spi receive timeout\n");
				break;
			}   
		}   

		buf[i] = (uint8_t)SPI_I2S_ReceiveData(spi_bus[minor]);
	}

    return i;
}

struct hal_bus spi2 = {
	.name = "spi2",
	.use_dma = false,
	.use_int = false,
	.bus = BUS(SPI, 1),
	.io_init = spi_io_init,
	.request_dma = spi_request_dma,
	.bus_enable = spi_bus_enable,
	.bus_cfg = spi_bus_cfg,
	.xfer = spi_bus_xfer,
};

int spi_bus_init()
{
	hal_bus_register(&spi2);
	return 0;
}

hal_driver_init(spi_bus_init);
