#ifndef _BUS_H_
#define _BUS_H_
#include "global_includes.h"
#include "stm32f4xx.h"

typedef enum {
	UART	= 0,
	SPI		= 1,
	I2C		= 2,
} bus_t;

struct bus {
	char *name;
	uint8_t initilised:1,
			use_dma:1,
			use_int:1;

	uint8_t bn;
	void *priv;
	int (*io_init)(uint8_t bn);
	int (*request_dma)(uint8_t bn);
	int (*bus_cfg)(uint8_t bn, void *cfg);
	int (*xfer)(uint8_t bn, char *buf, uint32_t len, uint32_t dir);
};

enum {
	IN = 0,
	OUT,
};

#define BUS(type, sub) ((type << 4 | sub & 0x0f) & 0xff)

int bus_register(struct bus *bus);
int bus_init(uint8_t bus);
int bus_xfer(uint8_t bus, char *buf, uint32_t len, uint32_t dir);
#endif
