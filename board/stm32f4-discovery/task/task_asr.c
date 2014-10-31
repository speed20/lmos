#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "asr/ld3320.h"
#include "asr/sound.h"

static portTASK_FUNCTION_PROTO(vASRTestTask, pvParameters);

void vStartASRTestTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vASRTestTask, (signed char *)"ASR", 1024, NULL, uxPriority, (TaskHandle_t *)NULL);
}

enum {
	LEADER = 0,
	COMMAND,
};

static portTASK_FUNCTION(vASRTestTask, pvParameters)
{
	uint8_t nAsrRes;
	uint8_t count, mode, retry;
	uint8_t status, **p;
	uint8_t *command_prefix[] = {
		"ao ba ma",
		"tu dou",
		"tu ou",
		"sa mu",
		"a mu",
		"sa",
	};

	uint8_t *command[] = {
		"kai kong tiao",
		"guan kong tiao",
		"kai deng",
		"guan deng",
		"ai kong tiao",
		"an kong tiao",
		"ai deng",
		"an deng",
	};

	ld3320_init();

	/*
	if (ld3320_test() != 1) {
		vParTestSetLED(LED1, 1);
		return ;
	}
	*/

	xSemaphoreTake(xMP3Semaphore, 0);

#define ONLY_MP3

#ifdef ONLY_MP3
	for (;;) {
		printk("play\n");
		play_sound(ready_sound, sizeof(ready_sound));
		xSemaphoreTake(xMP3Semaphore, portMAX_DELAY);
	}
#endif

#if 1
	mode = LEADER;

	xSemaphoreTake(xASRSemaphore, 0);

	nAsrStatus = LD_ASR_NONE;

	for (;;) {
		nAsrStatus = LD_ASR_RUNING;
		if (mode == LEADER) {
			printk("wait for call\n");
			p = command_prefix;
			count = sizeof(command_prefix)/sizeof(command_prefix[0]);

		} else {
			printk("wait for command\n");
			retry--;
			p = command;
			count = sizeof(command)/sizeof(command[0]);
		}

		if (ld3320_run_ASR(p, count) == 0) {
			nAsrStatus = LD_ASR_ERROR;
		} else {
			xSemaphoreTake(xASRSemaphore, portMAX_DELAY);
		}

		switch (nAsrStatus) {
			case LD_ASR_RUNING:
				printk("running\n");
			case LD_ASR_ERROR:
				printk("error\n");
				break;
			case LD_ASR_FOUNDOK:
			{
				printk("find\n");
				nAsrRes = ld3320_GetResult();
				if(nAsrRes < count) {					
					printk("prefix result: %s\n", p[nAsrRes]);

					if (mode == LEADER && nAsrRes < 2) {
						play_sound(ready_sound, sizeof(ready_sound));
						xSemaphoreTake(xMP3Semaphore, portMAX_DELAY);
						mode = COMMAND;
						retry = 3;
					} else if (mode == COMMAND && nAsrRes < 4) {
						play_sound(find_sound, sizeof(find_sound));
						xSemaphoreTake(xMP3Semaphore, portMAX_DELAY);
						mode = LEADER;
					}
				}
				nAsrStatus = LD_ASR_NONE;				
				break;
			}
			case LD_ASR_FOUNDZERO:
			default:
			{
				printk("not found\n");
				nAsrStatus = LD_ASR_NONE;
				if (retry <= 0 || retry > 3)
					mode = LEADER;
				break;
			}
		}
	}
#endif
}
