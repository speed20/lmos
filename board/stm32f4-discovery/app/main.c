/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* Demo application includes. */
#include "led.h"

/* Hardware and starter kit includes. */
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "serial.h"
#include "usbd_hid_core.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#include "mpulib.h"
#include <display/oled.h>
#include "delay.h"
//#include "bt.h"


/* Priorities for the demo application tasks. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainCREATOR_TASK_PRIORITY			( tskIDLE_PRIORITY + 3UL )
#define mainFLOP_TASK_PRIORITY				( tskIDLE_PRIORITY )
#define mainIR_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

/* The LED used by the check timer. */
#define mainCHECK_LED 						( 3UL )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK						( 0UL )

/* The period after which the check timer will expire, in ms, provided no errors
have been reported by any of the standard demo tasks.  ms are converted to the
equivalent in ticks using the portTICK_RATE_MS constant. */
#define mainCHECK_TIMER_PERIOD_MS			( 3000UL / portTICK_RATE_MS )

/* The period at which the check timer will expire, in ms, if an error has been
reported in one of the standard demo tasks.  ms are converted to the equivalent
in ticks using the portTICK_RATE_MS constant. */
#define mainERROR_CHECK_TIMER_PERIOD_MS 	( 200UL / portTICK_RATE_MS )

/* Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 1 to create a simple demo.
Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 0 to create a much more
comprehensive test application.  See the comments at the top of this file, and
the documentation page on the http://www.FreeRTOS.org web site for more
information. */
#define mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY		1

/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

/*
 * The check timer callback function, as described at the top of this file.
 */
static void prvCheckTimerCallback( xTimerHandle xTimer );

/*
 * Configure the interrupts used to test the interrupt nesting depth as
 * described at the top of this file.
 */
static void prvSetupNestedFPUInterruptsTest( void );

/*
 * The task that is synchronised with the button interrupt.  This is done just
 * to demonstrate how to write interrupt service routines, and how to
 * synchronise a task with an interrupt.
 */
static void prvButtonTestTask( void *pvParameters );

/*
 * This file can be used to create either a simple LED flasher example, or a
 * comprehensive test/demo application - depending on the setting of the
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY constant defined above.  If
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to 1, then the following
 * function will create a lot of additional tasks and a software timer.  If
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to 0, then the following
 * function will do nothing.
 */
static void prvOptionallyCreateComprehensveTestApplication( void );

/* delay */
static uint32_t USBConfig(void);
static void USB_Test(void);
static void Fail_Handler(void);
static void TIM2_Config(void);
static void TIM5_Config(void);
static int mpu6050_config(void);

/*-----------------------------------------------------------*/

/* The following variables are used to verify that the interrupt nesting depth
is as intended.  ulFPUInterruptNesting is incremented on entry to an interrupt
that uses the FPU, and decremented on exit of the same interrupt.
ulMaxFPUInterruptNesting latches the highest value reached by
ulFPUInterruptNesting.  These variables have no other purpose. */
volatile unsigned long ulFPUInterruptNesting = 0UL, ulMaxFPUInterruptNesting = 0UL;

/* The semaphore used to demonstrate a task being synchronised with an
interrupt. */
xSemaphoreHandle xTestSemaphore = NULL;
xSemaphoreHandle mpu6050Semaphore = NULL;
//xSemaphoreHandle xIRSemaphore = NULL;

/* The variable that is incremented by the task synchronised with the button
interrupt. */
volatile unsigned long ulButtonPressCounts = 0UL;
USB_OTG_CORE_HANDLE  USB_OTG_dev;
uint8_t DemoEnterCondition = 0x00;
xQueueHandle xIRQueue = NULL;

/*-----------------------------------------------------------*/
int main(void)
{
	/* Configure the hardware ready to run the test. */
	prvSetupHardware();

//	TIM2_Config();
//	TIM5_Config();

	serial_println("hardware setup ok");

//	xIRQueue = xQueueCreate(10, sizeof(uint32_t));

	/* Start standard demo/test application flash tasks.  See the comments at
	the top of this file.  The LED flash tasks are always created.  The other
	tasks are only created if mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to
	0 (at the top of this file).  See the comments at the top of this file for
	more information. */
	//vStartLEDFlashTasks(mainFLASH_TASK_PRIORITY);
	vStartPulseTask(mainFLOP_TASK_PRIORITY);
	//vStartMPU6050Tasks(mainFLOP_TASK_PRIORITY);
	//vStartIRTestTask(mainIR_TASK_PRIORITY);
	//vStartASRTestTask(mainIR_TASK_PRIORITY);

	/* The following function will only create more tasks and timers if
	mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to 0 (at the top of this
	file).  See the comments at the top of this file for more information. */
	prvOptionallyCreateComprehensveTestApplication();
	serial_println("system ready to run...");

	/* Start the scheduler. */
	vTaskStartScheduler();
	
	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	serial_print("fatal error!\r\n");
	for( ;; );	
}
/*-----------------------------------------------------------*/

static void prvCheckTimerCallback( xTimerHandle xTimer )
{
static long lChangedTimerPeriodAlready = pdFALSE;
long lErrorFound = pdFALSE;

	/* Check all the demo tasks (other than the flash tasks) to ensure
	that they are all still running, and that none have detected an error. */

	if( xAreMathsTaskStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if( xAreDynamicPriorityTasksStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if( xAreBlockingQueuesStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if ( xAreBlockTimeTestTasksStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if ( xAreGenericQueueTasksStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if ( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if( xIsCreateTaskStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if( xArePollingQueuesStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}

	if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	{
		lErrorFound = pdTRUE;
	}
	
	/* Toggle the check LED to give an indication of the system status.  If
	the LED toggles every mainCHECK_TIMER_PERIOD_MS milliseconds then
	everything is ok.  A faster toggle indicates an error. */
	vParTestToggleLED( mainCHECK_LED );	
	
	/* Have any errors been latch in lErrorFound?  If so, shorten the
	period of the check timer to mainERROR_CHECK_TIMER_PERIOD_MS milliseconds.
	This will result in an increase in the rate at which mainCHECK_LED
	toggles. */
	if( lErrorFound != pdFALSE )
	{
		if( lChangedTimerPeriodAlready == pdFALSE )
		{
			lChangedTimerPeriodAlready = pdTRUE;
			
			/* This call to xTimerChangePeriod() uses a zero block time.
			Functions called from inside of a timer callback function must
			*never* attempt	to block. */
			xTimerChangePeriod( xTimer, ( mainERROR_CHECK_TIMER_PERIOD_MS ), mainDONT_BLOCK );
		}
	}
}
/*-----------------------------------------------------------*/

static void prvButtonTestTask( void *pvParameters )
{
	configASSERT( xTestSemaphore );

	/* This is the task used as an example of how to synchronise a task with
	an interrupt.  Each time the button interrupt gives the semaphore, this task
	will unblock, increment its execution counter, then return to block
	again. */
	
	/* Take the semaphore before started to ensure it is in the correct
	state. */
	xSemaphoreTake( xTestSemaphore, mainDONT_BLOCK );
	
	for(;;)
	{
		xSemaphoreTake( xTestSemaphore, portMAX_DELAY );
		vParTestToggleLED(LED3);
		ulButtonPressCounts++;
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

	serial_init(USART_PORT, BAUDRATE);

//	i2c_init(0, 400000);

	USBConfig();
	
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Setup the LED outputs. */
	vParTestInitialise();
	
	/* Configure the button input.  This configures the interrupt to use the
	lowest interrupt priority, so it is ok to use the ISR safe FreeRTOS API
	from the button interrupt handler. */
	STM_EVAL_PBInit( BUTTON_USER, BUTTON_MODE_EXTI );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	#if ( mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY == 0 )
	{
		/* Just to verify that the interrupt nesting behaves as expected,
		increment ulFPUInterruptNesting on entry, and decrement it on exit. */
		ulFPUInterruptNesting++;

		/* Trigger a timer 2 interrupt, which will fill the registers with a
		different value and itself trigger a timer 3 interrupt.  Note that the
		timers are not actually used.  The timer 2 and 3 interrupt vectors are
		just used for convenience. */
		NVIC_SetPendingIRQ( TIM2_IRQn );
	
		ulFPUInterruptNesting--;
	}
	#endif
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

static void prvOptionallyCreateComprehensveTestApplication( void )
{
	#if ( mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY == 0 )
	{
		xTimerHandle xCheckTimer = NULL;

		/* Configure the interrupts used to test FPU registers being used from
		nested interrupts. */
		prvSetupNestedFPUInterruptsTest();

		/* Start all the other standard demo/test tasks. */
		vStartIntegerMathTasks( tskIDLE_PRIORITY );
		vStartDynamicPriorityTasks();
		vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
		vCreateBlockTimeTasks();
		vStartCountingSemaphoreTasks();
		vStartGenericQueueTasks( tskIDLE_PRIORITY );
		vStartRecursiveMutexTasks();
		vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
		vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );

		/* Most importantly, start the tasks that use the FPU. */
		vStartMathTasks( mainFLOP_TASK_PRIORITY );
		
		/* Create the semaphore that is used to demonstrate a task being
		synchronised with an interrupt. */
		vSemaphoreCreateBinary( xTestSemaphore );

		/* Create the task that is unblocked by the demonstration interrupt. */
		xTaskCreate( prvButtonTestTask, ( signed char * ) "BtnTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
		
		/* Create the software timer that performs the 'check' functionality,
		as described at the top of this file. */
		xCheckTimer = xTimerCreate( ( const signed char * ) "CheckTimer",/* A text name, purely to help debugging. */
									( mainCHECK_TIMER_PERIOD_MS ),		/* The timer period, in this case 3000ms (3s). */
									pdTRUE,								/* This is an auto-reload timer, so xAutoReload is set to pdTRUE. */
									( void * ) 0,						/* The ID is not used, so can be set to anything. */
									prvCheckTimerCallback				/* The callback function that inspects the status of all the other tasks. */
								  );	
		
		if( xCheckTimer != NULL )
		{
			xTimerStart( xCheckTimer, mainDONT_BLOCK );
		}

		/* This task has to be created last as it keeps account of the number of
		tasks it expects to see running. */
		vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );
	}
	#else /* mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY */
	{
		/* Create the semaphore that is used to demonstrate a task being
		synchronised with an interrupt. */
		vSemaphoreCreateBinary( xTestSemaphore );

		/* Create the task that is unblocked by the demonstration interrupt. */
		xTaskCreate( prvButtonTestTask, ( signed char * ) "BtnTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

		/* Just to prevent compiler warnings when the configuration options are
		set such that these static functions are not used. */
//		( void ) prvCheckTimerCallback;
//		( void ) prvSetupNestedFPUInterruptsTest;
	}
	#endif /* mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; ) {
		vParTestToggleLED(1);
	};
}
/*-----------------------------------------------------------*/
/**
  * @brief  Initializes the USB for the demonstration application.
  * @param  None
  * @retval None
  */
static uint32_t USBConfig(void)
{
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
   //         &USBD_HID_cb,
			&USBD_CDC_cb,
            &USR_cb);
  
  return 0;
} 

void TIM2_Config()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Enable the TIM2 interrupt in the NVIC.  The timer itself is not used,
	just its interrupt vector to force nesting from software.  TIM2 must have
	a lower priority than TIM3, and both must have priorities above
	configMAX_SYSCALL_INTERRUPT_PRIORITY. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	TIM_DeInit(TIM2);
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xffff; // 1 MHz down to 1 KHz (1 ms)
	//TIM_TimeBaseStructure.TIM_Prescaler = 840 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_Prescaler = 10 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM IT enable */
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

static void TIM5_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* --------------------------- System Clocks Configuration -----------------*/
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM5 pins to AF2 */  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	TIM_DeInit(TIM5);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; /* 84: 0.001ms, 840: 0.01ms, 8400: 0.1ms */
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	/* Enable TIM5 Preload register on ARR */
	TIM_ARRPreloadConfig(TIM5, ENABLE);

	/* TIM Capture Mode configuration: Channe2 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge; //TIM_ICPolarity_Falling
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0xff;

	/* Output Compare PWM1 Mode configuration: Channel1 */
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	/* TIM IT enable */
	TIM_Cmd(TIM5, DISABLE);
	TIM_ClearFlag(TIM5, TIM_FLAG_CC2);
	TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);

	/* TIM5 enable counter */
	TIM_Cmd(TIM5, ENABLE);
}

static int mpu6050_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef myNVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	vSemaphoreCreateBinary(mpu6050Semaphore);
	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    myNVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    myNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
    myNVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    myNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&myNVIC_InitStructure);

	//mpu6050_initialize();
	//mpu6050_dmpInitialize();
	/*
	mpu6050_setXGyroOffset(220);
	mpu6050_setYGyroOffset(76);
	mpu6050_setZGyroOffset(-85);
	mpu6050_setZAccelOffset(1788);
	*/
}
