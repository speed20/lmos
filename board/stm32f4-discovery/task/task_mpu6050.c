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

extern USB_OTG_CORE_HANDLE  USB_OTG_dev;
extern SemaphoreHandle_t mpu6050Semaphore;
static portTASK_FUNCTION_PROTO(vMPU6050TestTask, pvParameters);

void vStartMPU6050Tasks(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vMPU6050TestTask, (signed char *)"imu", STACK_SIZE, NULL, uxPriority, (TaskHandle_t *)NULL);
}

#pragma pack(1) 
struct mpu6050_report_t {
	int32_t x, y, z;
	uint8_t button;
};

#define SCALE 2000
static portTASK_FUNCTION(vMPU6050TestTask, pvParameters)
{
	TickType_t xSendRate, xLastSendTime;
	xSendRate = 2;
	int16_t ax, ay, az, rx, ry, rz, hx, hy, hz;
	int32_t roll, yaw, pitch;
	unsigned char buf[128];
	struct mpu6050_report_t report;
	uint32_t t1, t2, dt, i;
	uint16_t packetSize, fifoCount;
	uint8_t mpuIntStatus, fifoBuffer[64];
	Quaternion q;
	VectorFloat gravity;
	VectorInt16 raw, aa;
	float ypr[3];
	mpudata_t mpu;
	uint8_t empty[16]= {' '};

	memset(&report, 0, sizeof(report));
	memset(&mpu, 0, sizeof(mpu));

	oled_clear();

	xSemaphoreTake(mpu6050Semaphore, 0);

	if (mpulib_init(50, 10) < 0) {
		for(;;);
	}

//	set_calibration(0);
//	set_calibration(1);

	uint32_t size;
	for (;;)
	{
		xSemaphoreTake(mpu6050Semaphore, portMAX_DELAY);
		if (mpulib_read(&mpu) == 0) {
			//sprintf(buf, "%d,%d,%d\r\n", mpu.rawAccel[0], mpu.rawAccel[1], mpu.rawAccel[2]);
			size = sprintf(buf, "%d,%d,%d\r\n", (int32_t)((mpu.fusedEuler[VEC3_X]) * RAD_TO_DEGREE), \
					(int32_t)((mpu.fusedEuler[VEC3_Y]) * RAD_TO_DEGREE), \
					(int32_t)((mpu.fusedEuler[VEC3_Z]) * RAD_TO_DEGREE));

//			sprintf(buf, "%d,%d,%d\r\n", mpu.rawMag[0], mpu.rawMag[1], mpu.rawMag[2]);
//			VCP_send_str(buf);
			vParTestToggleLED(LED3);
			Bt_Send(buf, size);
			serial_print("%s", buf);
			buf[size-2] = '\0';
			oled_show_string(0, 0, buf);
			//serial_print("%d %d %d\r\n", mpu.rawAccel[0], mpu.rawAccel[1], mpu.rawAccel[2]);
			//serial_print("%d %d %d\r\n", m_calAccel[0], m_calAccel[1], m_calAccel[2]);
		}
	}

#if 0
	t1 = TIM_GetCounter(TIM2);
	for (i=0; i<1000000; i++) {
		USBD_HID_SendReport(&USB_OTG_dev, &report, sizeof(report));
	}
	t2 = TIM_GetCounter(TIM2);
	dt = t2 - t1 >= 0 ? t2 - t1 : t2 - t1 + 0xffff;
	serial_print("usb speed: %d %d Mbytes/s\r\n", dt, sizeof(report) * 100000 / dt);
	//serial_print("usb speed: %d %d kbytes/s\r\n", dt, sizeof(report) * 10000 * 1000 * 100 / dt);
	while (1);
#endif
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
