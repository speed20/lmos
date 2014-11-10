#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal.h"

static portTASK_FUNCTION_PROTO(vPmonTask, pvParameters);

void vStartPmonTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vPmonTask, (signed char *)"flash_pmon", 1024, NULL, uxPriority, (TaskHandle_t *)NULL);
}

static portTASK_FUNCTION(vPmonTask, pvParameters)
{
	bus_t spi = BUS(SPI, 1);
	bus_t usb_vcp = BUS(USB, 0);

	char buf[] = {0x01, 0x02};

	printk("pmon task start!\n");

	hal_bus_xfer(spi, 0, buf, 2, OUT);

	for (;;) {
		hal_bus_xfer(usb_vcp, 0, "hello\r\n", 7, OUT);
	}
}
