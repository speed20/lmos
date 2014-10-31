#ifndef _BUS_H_
#define _BUS_H_

#include "global_includes.h"
#include "stm32f4xx.h"

typedef enum {
	UART	= 0,
	SPI		= 1,
	I2C		= 2,
} major_t;

typedef enum {
	IN = 0,
	OUT,
} DIRECTION;

typedef unsigned char bus_t;

struct hal_bus {
	char *name;
	uint8_t initilised:1,
			enabled,
			use_dma:1,
			use_int:1;

	bus_t bus;
	void *priv;
	int (*io_init)(bus_t bus);				/* used to do io init, pin map, config gpio, etc.*/
	int (*request_dma)(bus_t bus);			/* if use_dma is true, use this function to request dma chnnal */
	int (*request_irq)(bus_t bus);
	int (*bus_enable)(bus_t bus, void *arg);
	int (*bus_cfg)(bus_t bus, void *cfg);
	int (*xfer)(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir); /* do data transfer */
};

int hal_bus_register(struct hal_bus *hbus);
int hal_bus_enable(bus_t bus, void *arg);
int hal_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir);

typedef int (*initcall_t)(void);

#define BUS(major, minor) ((major << 4 | minor & 0x0f) & 0xff)

#define __define_initcall(level, fn, id) \
	    initcall_t __initcall_##fn __attribute__((section(".initcall"level".init"))) = fn

#define hal_driver_init(fn)     __define_initcall("6", fn, 6)

void hal_init();
#endif
