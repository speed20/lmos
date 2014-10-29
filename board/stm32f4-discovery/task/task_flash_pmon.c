#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static portTASK_FUNCTION_PROTO(vFlashPmonTask, pvParameters);

void vStartASRTestTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vFlashPmonTask, (signed char *)"flash_pmon", 1024, NULL, uxPriority, (TaskHandle_t *)NULL);
}

static portTASK_FUNCTION(vFlashPmonTask, pvParameters)
{
}
