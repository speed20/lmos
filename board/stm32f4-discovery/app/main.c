/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* Demo application includes. */
#include "led/led.h"

/* Hardware and starter kit includes. */
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "serial.h"

#include <display/oled.h>
#include "delay.h"
#include "display.h"
#include "arm_math.h"

/* Priorities for the demo application tasks. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainCREATOR_TASK_PRIORITY			( tskIDLE_PRIORITY + 3UL )
#define mainFLOP_TASK_PRIORITY				( tskIDLE_PRIORITY )
#define mainIR_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

#define mainCHECK_LED 						( 3UL )
#define mainDONT_BLOCK						( 0UL )
#define mainCHECK_TIMER_PERIOD_MS			( 3000UL / portTICK_RATE_MS )
#define mainERROR_CHECK_TIMER_PERIOD_MS 	( 200UL / portTICK_RATE_MS )

#define mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY		1

/*-----------------------------------------------------------*/
static void prvSetupHardware( void );
static void prvCheckTimerCallback( TimeOut_t xTimer );

int main(void)
{
	prvSetupHardware();

	vStartLEDFlashTasks(mainFLASH_TASK_PRIORITY);
	vStartPulseTask(mainCREATOR_TASK_PRIORITY);
	//vStartMPU6050Tasks(mainFLOP_TASK_PRIORITY);
	//vStartIRTestTask(mainIR_TASK_PRIORITY);
	//vStartASRTestTask(mainIR_TASK_PRIORITY);

	serial_println("system ready to run...");

	vTaskStartScheduler();
	serial_print("fatal error!\r\n");
	for(;;);	
}
/*-----------------------------------------------------------*/

static void prvCheckTimerCallback( TimeOut_t xTimer )
{
}

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	vParTestInitialise();
	serial_init(PRINT_PORT, BAUDRATE);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
//	i2c_init(0, 400000);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/

static void prvSetupNestedFPUInterruptsTest( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 interrupt in the NVIC.  The timer itself is not used,
	just its interrupt vector to force nesting from software.  TIM2 must have
	a lower priority than TIM3, and both must have priorities above
	configMAX_SYSCALL_INTERRUPT_PRIORITY. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	TIM2_Config();

#if 0
	/* Enable the TIM3 interrupt in the NVIC.  The timer itself is not used,
	just its interrupt vector to force nesting from software.  TIM2 must have
	a lower priority than TIM3, and both must have priorities above
	configMAX_SYSCALL_INTERRUPT_PRIORITY. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
#endif
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	taskDISABLE_INTERRUPTS();
	for( ;; ) {
		vParTestToggleLED(1);
	};
}
