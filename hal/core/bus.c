/* serial-likely bus, include spi/i2c/uart/usart */
#include "bus.h"
#include "init.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#define MAX_BUS	128

struct bus *g_serial_list[MAX_BUS] = {NULL};

int bus_register(struct bus *bus)
{
	if (bus == NULL) {
		return -ENODEV;
	}

	if (g_serial_list[bus->bn] != NULL)
		return -EEXIST;

	g_serial_list[bus->bn] = bus;

	if (bus_init(bus->bn) < 0) {
		serial_println("bus %s not registered\n", bus->name);
		return -1;
	};

	serial_println("bus %s registered\n", bus->name);
	return 0;
}

int bus_init(uint8_t bn)
{
	struct bus *bus = g_serial_list[bn];
	serial_println("%s %d", __func__, __LINE__);

	if (!bus) {
		serial_println("bus %d not registered\n", bn);
		return -ENODEV;
	}

	if (bus->io_init(bus->bn) < 0)
		return -1;

	serial_println("use dma: %d\n", bus->use_dma);

	if (bus->use_dma) {
		if (bus->request_dma(bus->bn) < 0)
			return -1;
	}
	
	return 0;
}

/*
void serial_cs(uint8_t bn, bool enable)
{
	struct bus *bus = g_serial_list[num];

	if (!bus) {
		serial_println("bus %d not registered\n");
	} else {
		bus->cs_enable(enable);
	}
}
*/

int bus_xfer(uint8_t bn, char *buf, uint32_t len, uint32_t dir)
{
	struct bus *bus = g_serial_list[bn];

	if (!bus) {
		serial_println("bus %d not registered\n", bn);
		return -ENODEV;
	}

	if (bus->xfer)
		return bus->xfer(bn, buf, len, dir);
	else
		return -ENOSYS;
}
