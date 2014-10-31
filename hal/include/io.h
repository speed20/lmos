#ifndef __PIN_MAP_H
#define __PIN_MAP_H
#include "stm32f4xx.h"

typedef enum {
	AHB1 = 0,
	AHB2 = 1,
	AHB3 = 2,
	APB1 = 3,
	APB2 = 4,
}clk_flag_t;

struct io_map {
	char *name;
	uint16_t pin;
	uint8_t src;
	GPIO_TypeDef *port;
	clk_flag_t clk_flag;
	uint32_t clk;
	uint8_t alt;
	GPIOPuPd_TypeDef pull;
	GPIOOType_TypeDef otype;
	GPIOSpeed_TypeDef speed;
	GPIOMode_TypeDef mode;
};

void io_request(const struct io_map *map);
#endif
