#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

static portTASK_FUNCTION_PROTO(vPowerboxTask, pvParameters);

void vStartPowerboxTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vPowerboxTask, (signed char *)"Powerbox", 1024, NULL, uxPriority, (TaskHandle_t *)NULL);
}


static portTASK_FUNCTION(vPowerboxTask, pvParameters)
{
	uint8_t resp1[6] = {0x48, 0x54, 0x01, 0x00, 0x00, 0x00};
	uint8_t resp2[6] = {0x4c, 0x47, 0x01, 0x00, 0x00, 0x00};
	uint8_t unsol_resp[8] = {0x48, 0x54, 0x03, 0x80, 0x02, 0x00, 0x00, 0x21};
	int count = 0;
	uint8_t buf[8], lastc, newc;

	lastc = 0;
	for (;;) {
		serial_read(0, &newc, 1);
		if (lastc == 'H' && newc == 'T') {
			Delay(10000);
			serial_write(0, resp1, 6);
		} else if (lastc == 'L' && newc == 'G') {
			Delay(10000);
			serial_write(0, resp2, 6);
		}
		lastc = newc;
#if 0
		if (count % 2 == 0) {
			Delay(10000);
			serial_write(0, unsol_resp, 8);
		}
#endif
		count++;
	}
}

