#include "stdint.h"
#include "global_includes.h"
#include "stm32f4xx.h"
#include "io.h"

void io_request(const struct io_map *map)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	switch(map->clk_flag) {
		case AHB1:
			RCC_AHB1PeriphClockCmd(map->clk, ENABLE);
			break;
		case AHB2:
			RCC_AHB2PeriphClockCmd(map->clk, ENABLE);
			break;
		case AHB3:
			RCC_AHB3PeriphClockCmd(map->clk, ENABLE);
			break;
		case APB1:
			RCC_APB1PeriphClockCmd(map->clk, ENABLE);
			break;
		case APB2:
			RCC_APB2PeriphClockCmd(map->clk, ENABLE);
			break;
	}

	if (map->alt != -1)
		GPIO_PinAFConfig(map->port, map->src, map->alt);

	GPIO_InitStructure.GPIO_Pin = map->pin;
	GPIO_InitStructure.GPIO_Mode = map->mode;
	GPIO_InitStructure.GPIO_OType = map->otype;
	GPIO_InitStructure.GPIO_PuPd  = map->pull;
	GPIO_InitStructure.GPIO_Speed = map->speed;
	GPIO_Init(map->port, &GPIO_InitStructure);
}
