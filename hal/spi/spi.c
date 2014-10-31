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
	serial_println("%s %d", __func__, __LINE__);
	int ret = 0;

    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	if (((bus&0xf0) >> 4) != SPI)
		return -1;

	serial_println("%s %d", __func__, __LINE__);

	switch(bus&0x0f) {
		case 0:
			{
				serial_println("not implement");
				ret = -1;
				break;
			}
		case 1:
			{
				int i;
				for (i=0; i<3; i++) {
					io_request(&spi2_io_map[i]);
				}
				serial_println("%s %d", __func__, __LINE__);
				break;
			}
		case 2:
			{
				serial_println("not implement");
				ret = -1;
				break;
			}
	}

	serial_println("%s %d", __func__, __LINE__);
	return ret;
}

int spi_request_dma(bus_t bus)
{
	serial_println("spi request dma, bus: %d", bus);
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
	serial_println("%s %d bytes\n", dir == IN ? "receive" : "send", len);
    uint32_t timeout;
    uint8_t val;
	int i;

	for (i=0; i<len; i++) {
		timeout = 0x1000;
		while (SPI_I2S_GetFlagStatus(spi_bus[bus&0xf], SPI_I2S_FLAG_TXE) == RESET) {
			if(timeout-- == 0) {
				serial_println("spi send timeout");
				break;
			}   
		}   

		SPI_I2S_SendData(spi_bus[bus&0xf], buf[i]);

		timeout = 0x1000;
		while (SPI_I2S_GetFlagStatus(spi_bus[bus&0xf], SPI_I2S_FLAG_RXNE) == RESET) {
			if(timeout-- == 0) {
				serial_println("spi receive timeout");
				break;
			}   
		}   

		buf[i] = (uint8_t)SPI_I2S_ReceiveData(spi_bus[bus&0xf]);
	}

    return i;
}

struct hal_bus spi0 = {
	.name = "spi0",
	.use_dma = true,
	.bus = BUS(SPI, 1),
	.io_init = spi_io_init,
	.request_dma = spi_request_dma,
	.bus_enable = spi_bus_enable,
	.bus_cfg = spi_bus_cfg,
	.xfer = spi_bus_xfer,
};

int spi_bus_init()
{
	hal_bus_register(&spi0);
	return 0;
}

hal_driver_init(spi_bus_init);
