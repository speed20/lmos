#include <stdio.h>
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 4 )
#define ledFLASH_RATE_BASE	( ( portTickType ) 125)

static portTASK_FUNCTION_PROTO(vLEDFlashTask, pvParameters);
static xTaskHandle led_handle[ledNUMBER_OF_LEDS];
static volatile unsigned portBASE_TYPE uxFlashTaskNumber = 0;

void vStartLEDFlashTasks( unsigned portBASE_TYPE uxPriority )
{
	signed portBASE_TYPE xLEDTask;

	/* Create the three tasks. */
	for(xLEDTask=0; xLEDTask<ledNUMBER_OF_LEDS; ++xLEDTask) {
		/* Spawn the task. */
		xTaskCreate( vLEDFlashTask, ( signed char * ) "LEDx", ledSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * )&led_handle[xLEDTask]);
	}
}

static portTASK_FUNCTION(vLEDFlashTask, pvParameters)
{
	portTickType xFlashRate, xLastFlashTime;
	unsigned portBASE_TYPE uxLED;
	uint8_t tmp;

	/* The parameters are not used. */
	( void ) pvParameters;

	/* Calculate the LED and flash rate. */
	portENTER_CRITICAL();

	/* See which of the eight LED's we should use. */
	uxLED = uxFlashTaskNumber;

	/* Update so the next task uses the next LED. */
	uxFlashTaskNumber++;

	portEXIT_CRITICAL();

	xFlashRate = ledFLASH_RATE_BASE + ( ledFLASH_RATE_BASE * ( portTickType ) uxLED );
	xFlashRate /= portTICK_RATE_MS;

	/* We will turn the LED on and off again in the delay period, so each
	delay is only half the total period. */
	xFlashRate /= ( portTickType ) 2;

	/* We need to initialise xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		vTaskDelayUntil( &xLastFlashTime, xFlashRate );
		vParTestToggleLED( uxLED );
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */
