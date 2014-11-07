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

	if (hal_bus_init(hbus) < 0) {
		printk("bus init failed, bus %s %d not registered\n", hbus->name, hbus->bus);
		hbus_list[hbus->bus] = NULL;
		return -1;
	};

	printk("bus %s registered\n", hbus->name);
	return 0;
}

int hal_bus_init(struct hal_bus *hbus)
{
	if (hbus->io_init(hbus->bus) < 0) {
		printk("%s io init failed\n", hbus->name);
		return -1;
	}

	if (hbus->use_dma) {
		printk("use dma: %d\n", hbus->use_dma);
		if (!hbus->request_dma || hbus->request_dma(hbus->bus) < 0) {
			printk("%s request_dma failed.\n", hbus->name);
			return -1;
		}
	}

	if (hbus->use_int) {
		if (!hbus->request_irq || hbus->request_irq(hbus->bus) < 0) {
			printk("%s request_irq failed.\n", hbus->name);
			return -1;
		}
	}

	if (!hbus->bus_enable || hbus->bus_enable(hbus->bus, hbus->priv) < 0) {
		printk("%s enable failed.", hbus->name);
		return -1;
	} else
		return 0;
}

int hal_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir)
{
	struct hal_bus *hbus = hbus_list[bus];

	if (!hbus) {
		printk("bus %d not registered\n", bus);
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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	for (fn = &__init_start; fn < &__init_end; fn++) {
		printk("fn: %p\n", fn);
		(*fn)();
	}
}
