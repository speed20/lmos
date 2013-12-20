#include <stdio.h>
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "usb_core.h"
#include "mpu6050.h"
#include "mpulib.h"
#include "arm_math.h"

#define STACK_SIZE 1024

extern USB_OTG_CORE_HANDLE  USB_OTG_dev;
extern xSemaphoreHandle mpu6050Semaphore;
static portTASK_FUNCTION_PROTO(vMPU6050TestTask, pvParameters);

void vStartMPU6050Tasks(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vMPU6050TestTask, (signed char *)"imu", STACK_SIZE, NULL, uxPriority, (xTaskHandle *)NULL);
}

#pragma pack(1) 
struct mpu6050_report_t {
	int32_t x, y, z;
	uint8_t button;
};

#define SCALE 2000
static portTASK_FUNCTION(vMPU6050TestTask, pvParameters)
{
	portTickType xSendRate, xLastSendTime;
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
			vParTestToggleLED(LED1);
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


