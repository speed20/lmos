#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4_discovery.h"
#include "ld3320.h"
//#include "DemoSound.h"
//#include "sonar.h"
#include "complete.h"

static portTASK_FUNCTION_PROTO(vASRTestTask, pvParameters);

void vStartASRTestTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vASRTestTask, (signed char *)"ASR", 1024, NULL, uxPriority, (xTaskHandle *)NULL);
}

void do_ASR()
{
}

static portTASK_FUNCTION(vASRTestTask, pvParameters)
{
	uint8_t nAsrRes;
	unsigned portBASE_TYPE uxLED;
	portTickType xFlashRate, xLastFlashTime;

	ld3320_init();

	/*
	if (ld3320_test() != 1) {
		vParTestSetLED(LED1, 1);
		return ;
	}
	*/

	xSemaphoreTake(xMP3Semaphore, 0);

#ifdef ONLY_MP3
	for (;;) {
		serial_println("play");
		play_sound(mp3_buf, sizeof(mp3_buf));
		//play_sound(bpDemoSound, DEMO_SOUND_SIZE); //sizeof(bpDemoSound));
		xSemaphoreTake(xMP3Semaphore, portMAX_DELAY);
	}
#endif

#if 1
	xSemaphoreTake(xASRSemaphore, 0);

	nAsrStatus = LD_ASR_NONE;

	for (;;) {
		vParTestSetLED(LED1, 1);

		nAsrStatus = LD_ASR_RUNING;
		if (RunASR() == 0) {
			nAsrStatus = LD_ASR_ERROR;
		} else {
			vParTestSetLED(LED1, 0);
			xSemaphoreTake(xASRSemaphore, portMAX_DELAY);
		}

		switch (nAsrStatus) {
			case LD_ASR_RUNING:
			case LD_ASR_ERROR:
				vParTestToggleLED(LED1);
				break;
			case LD_ASR_FOUNDOK:
			{
				nAsrRes = ld3320_GetResult();
				if(nAsrRes < ITEM_COUNT) {					
					serial_println("result: %s size: %d", str_pattern[nAsrRes], sizeof(mp3_buf));
					play_sound(mp3_buf, sizeof(mp3_buf));
					xSemaphoreTake(xMP3Semaphore, portMAX_DELAY);
				}
				nAsrStatus = LD_ASR_NONE;				
				break;
			}
			case LD_ASR_FOUNDZERO:
			default:
			{
				nAsrStatus = LD_ASR_NONE;
				break;
			}
		}
	}
#endif
}
