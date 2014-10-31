#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static portTASK_FUNCTION_PROTO(vIRTestTask, pvParameters);

extern uint8_t start;
extern SemaphoreHandle_t xIRSemaphore;
extern QueueHandle_t xIRQueue;

void vStartIRTestTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vIRTestTask, (signed char *)"IR", 1024, NULL, uxPriority, (TaskHandle_t *)NULL);
}

struct wave_data {
	uint8_t init_level;
	uint32_t ms[256];
	uint16_t counter;
};

static portTASK_FUNCTION(vIRTestTask, pvParameters)
{
	uint8_t i, counter;
	uint8_t bit[32];
	uint8_t flag, tmp;
	struct wave_data data;
	uint32_t val;

	counter = 0;
	flag = 0;

	if (xIRQueue == 0) {
		printk("queue error\n");
		while(1);
	}

	for(;;)
	{
		if (xQueueReceive(xIRQueue, (void *)&val, portTICK_PERIOD_MS * 100) == pdTRUE) {
			if (val >> 16 == 0xBEEF) {
				data.init_level = ~val & 0x01;
				data.counter = 0;
				data.ms[data.counter++] = 0;
				flag = 1;
			} else {
				data.ms[data.counter++] = val;
			}
		} else {
			if (flag) {
				/* report wave data */
				printk("total bits: %d\n", data.counter);
				tmp = data.init_level;
				for (i=0; i<data.counter; i++) {
					printk("%c:%d\n", tmp == 0 ? 'L' : 'H', data.ms[i]);
					tmp = ~tmp & 0x01;
				}
				printk("\n");
				flag = 0;
				portENTER_CRITICAL();
				start = 1;
				portEXIT_CRITICAL();
			}
		}
	}
}
