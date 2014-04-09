#include <stdio.h>
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "usb_core.h"
#include <invense/mpu6050.h>
#include <mpulib/mpulib.h>
#include "arm_math.h"

#define STACK_SIZE 1024

SemaphoreHandle_t mpu6050Semaphore;
static portTASK_FUNCTION_PROTO(vMPUTask, pvParameters);
static int mpu6050_config();

void vStartMPUTasks(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vMPUTask, (signed char *)"imu", STACK_SIZE, NULL, uxPriority, (TaskHandle_t *)NULL);
}

#pragma pack(1) 
struct mpu6050_report_t {
	int32_t x, y, z;
	uint8_t button;
};

#define SCALE 2000
static portTASK_FUNCTION(vMPUTask, pvParameters)
{
	int32_t roll, yaw, pitch;
	unsigned char buf[128];
	uint32_t size;
	mpudata_t mpu;

	memset(&mpu, 0, sizeof(mpu));
	mpu6050_config();

	xSemaphoreTake(mpu6050Semaphore, 0);

	if (mpulib_init(20, 10) < 0) {
		serial_println("mpulib init failed");
		for (;;);
	}
	serial_println("mpu config finish");

//	set_calibration(0);
//	set_calibration(1);

	for (;;) {
		xSemaphoreTake(mpu6050Semaphore, portMAX_DELAY);
		if (mpulib_read(&mpu) == 0) {
			size = sprintf(buf, "%d,%d,%d", (int32_t)((mpu.fusedEuler[VEC3_X]) * RAD_TO_DEGREE), \
					(int32_t)((mpu.fusedEuler[VEC3_Y]) * RAD_TO_DEGREE), \
					(int32_t)((mpu.fusedEuler[VEC3_Z]) * RAD_TO_DEGREE));
			serial_println("%s", buf);
		}
	}
}

static int mpu6050_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	vSemaphoreCreateBinary(mpu6050Semaphore);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);

	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	//mpu6050_initialize();
	//mpu6050_dmpInitialize();
	/*
	mpu6050_setXGyroOffset(220);
	mpu6050_setYGyroOffset(76);
	mpu6050_setZGyroOffset(-85);
	mpu6050_setZAccelOffset(1788);
	*/
}

void EXTI9_5_IRQHandler(void)
//void EXTI1_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;
	uint8_t status;

	status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);

	EXTI_ClearITPendingBit(EXTI_Line5);
	if (status == Bit_RESET) { 
		xSemaphoreGiveFromISR(mpu6050Semaphore, &lHigherPriorityTaskWoken );
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}
}
