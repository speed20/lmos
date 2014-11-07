#include "FreeRTOS.h"
#include "hal.h"
#include "semphr.h"
#include "io.h"

int xxx_io_init(bus_t bus)
{
}

int xxx_request_dma(bus_t bus)
{
}

int xxx_bus_enable(bus_t bus, void *arg)
{
}

int xxx_bus_cfg(bus_t bus, void *cfg)
{
}

int xxx_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir)
{
}

struct hal_bus xxx = {
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

int xxx_bus_init()
{
}

hal_driver_init(xxx_bus_init);
