/* serial-likely bus, include spi/i2c/uart/usart */
#include "hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#define MAX_BUS	128

struct hal_bus *hbus_list[MAX_BUS] = {NULL};
extern initcall_t __init_start, __init_end;

int hal_bus_register(struct hal_bus *hbus)
{
	if (hbus == NULL) {
		return -ENODEV;
	}

	if (hbus_list[hbus->bus] != NULL)
		return -EEXIST;

	hbus_list[hbus->bus] = hbus;

	if (hal_bus_init(hbus->bus) < 0) {
		serial_println("bus %s not registered\n", hbus->name);
		return -1;
	};

	serial_println("bus %s registered\n", hbus->name);
	return 0;
}

int hal_bus_init(struct hal_bus *hbus)
{
	if (hbus->io_init(hbus->bus) < 0)
		return -1;

	if (hbus->use_dma) {
		serial_println("use dma: %d\n", hbus->use_dma);
		if (hbus->request_dma(hbus->bus) < 0)
			return -1;
	}

	return 0;
}

int hal_bus_enable(bus_t bus, void *arg)
{
	struct hal_bus *hbus = hbus_list[bus];
	
	if (!hbus) {
		serial_println("bus %d not registered\n", bus);
		return -ENODEV;
	}

	if (hbus->enabled)
		return 0;

	if (hbus->bus_enable) {
		int ret = hbus->bus_enable(bus, arg);
		if (ret == 0)
			hbus->enabled = 1;
		return ret;
	} else
		return -ENOSYS;
}

/*
void serial_cs(bus_t bus, bool enable)
{
	struct hal_bus *bus = hbus_list[num];

	if (!bus) {
		serial_println("bus %d not registered\n");
	} else {
		bus->cs_enable(enable);
	}
}
*/

int hal_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir)
{
	struct hal_bus *hbus = hbus_list[bus];

	if (!hbus) {
		serial_println("bus %d not registered\n", bus);
		return -ENODEV;
	}

	if (hbus->xfer)
		return hbus->xfer(bus, addr, buf, len, dir);
	else
		return -ENOSYS;
}

void hal_init()
{
	initcall_t *fn;

	for (fn = &__init_start; fn < &__init_end; fn++) {
		serial_println("fn: %p", fn);
		(*fn)();
	}
}
