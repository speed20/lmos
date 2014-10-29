#include <stdio.h>
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//#include "usb_core.h"
#include <invense/mpu6050.h>
#include <mpulib/mpulib.h>
#include "GUI.h"

#define STACK_SIZE 2048
#define SCALE 2000
#define N_SAMPLE 240

SemaphoreHandle_t mpu6050Semaphore;
static portTASK_FUNCTION_PROTO(vMPUTask, pvParameters);
static int mpu6050_config();

void vStartMPUTasks(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vMPUTask, (signed char *)"imu", STACK_SIZE, NULL, uxPriority, (TaskHandle_t *)NULL);
}

struct draw_ctx {
	int16_t *roll;
	int16_t *yaw;
	int16_t *pitch;
	int16_t *accel[3];
	int16_t *gyro[3];

	uint16_t nr_point;
};

#define V_OFFSET 160

void scale(int16_t *in, int16_t *out, uint16_t len, uint16_t m, uint16_t n)
{
	int i;

	for (i=0; i<len; i++) {
		out[i] = in[i] * m / n;
	}
}

void mpu_draw_waveform(void *pdata)
{
	struct draw_ctx *ctx = (struct draw_ctx *)pdata;
	unsigned char buf[64];
	int16_t tmp[N_SAMPLE];

#if 1
	GUI_MULTIBUF_Begin();
	GUI_Clear();

	GUI_SetColor(GUI_RED);
	sprintf(buf, "roll: %d", ctx->roll[0]);
	GUI_DispStringAt(buf, 5, 10);
	scale(ctx->roll, tmp, ctx->nr_point, 160, 180);
	GUI_DrawGraph(tmp, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_BLUE);
	sprintf(buf, "pitch: %d", ctx->pitch[0]);
	GUI_DispStringAt(buf, 5, 20);
	scale(ctx->pitch, tmp, ctx->nr_point, 160, 180);
	GUI_DrawGraph(tmp, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_DARKGREEN);
	sprintf(buf, "yaw: %d", ctx->yaw[0]);
	GUI_DispStringAt(buf, 5, 30);
	scale(ctx->yaw, tmp, ctx->nr_point, 160, 180);
	GUI_DrawGraph(tmp, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_YELLOW);
	GUI_DispStringAt("accel x", 100, 10);
	scale(ctx->accel[0], tmp, ctx->nr_point, 160, 32767);
	GUI_DrawGraph(tmp, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_ORANGE);
	GUI_DispStringAt("accel y", 100, 20);
	scale(ctx->accel[1], tmp, ctx->nr_point, 160, 32767);
	GUI_DrawGraph(tmp, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_CYAN);
	GUI_DispStringAt("accel z", 100, 30);
	scale(ctx->accel[2], tmp, ctx->nr_point, 160, 32767);
	GUI_DrawGraph(tmp, ctx->nr_point, 0, V_OFFSET);
#endif

	GUI_MULTIBUF_End();
}

static portTASK_FUNCTION(vMPUTask, pvParameters)
{
	uint32_t size, i;
	mpudata_t mpu;
	int16_t roll[N_SAMPLE], yaw[N_SAMPLE], pitch[N_SAMPLE];
	int16_t gyro[3][N_SAMPLE], accel[3][N_SAMPLE];
	struct draw_ctx ctx = {roll, yaw, pitch, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], 1};
	uint32_t count;
	float vx, vy, vz, ax, ay, az, sx, sy, sz;
	uint8_t status;

	LCD_Init();
	LTDC_Cmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	GUI_Init();
	GUI_SetBkColor(GUI_TRANSPARENT);
	GUI_SetPenSize(1);

	mpu6050_config();

	xSemaphoreTake(mpu6050Semaphore, 0);

	if (mpulib_init(100, 5) < 0) {
		serial_println("mpulib init failed");
		for (;;);
	}

#if 0
	status = mpu_run_self_test(gyro, accel);

	for(i = 0; i<3; i++) {
		gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
		accel[i] *= 4096.f; //convert to +-8G
		accel[i] = accel[i] >> 16;
		gyro[i] = (long)(gyro[i] >> 16);
	}

	serial_println("gyro bias: %d %d %d status: 0x%02x", \
			(int16_t)gyro[0], (int16_t)gyro[1], (int16_t)gyro[2], status);

	serial_println("accel bias: %d %d %d status: 0x%02x", \
			(int16_t)accel[0], (int16_t)accel[1], (int16_t)accel[2], status);

	mpu_set_gyro_bias_reg(gyro);
	mpu_set_accel_bias_6050_reg(accel);
#endif

	count = 0;
	vx = vy = vz = .0f;
	sx = sy = sz = .0f;
	for (;;) {
		xSemaphoreTake(mpu6050Semaphore, portMAX_DELAY);
		if (mpulib_read(&mpu) == 0) {
			for (i=ctx.nr_point-1; i>0; i--) {
				roll[i] = roll[i-1];
				pitch[i] = pitch[i-1];
				yaw[i] = yaw[i-1];

				accel[0][i] = accel[0][i-1];
				accel[1][i] = accel[1][i-1];
				accel[2][i] = accel[2][i-1];

				gyro[0][i] = gyro[0][i-1];
				gyro[1][i] = gyro[1][i-1];
				gyro[2][i] = gyro[2][i-1];
			}

			roll[0] = mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE;
			yaw[0] = mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE;
			pitch[0] = mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE;

			accel[0][0] = mpu.rawAccel[VEC3_X];
			accel[1][0] = mpu.rawAccel[VEC3_Y];
			accel[2][0] = mpu.rawAccel[VEC3_Z];

			gyro[0][0] = mpu.rawGyro[VEC3_X];
			gyro[1][0] = mpu.rawGyro[VEC3_Y];
			gyro[2][0] = mpu.rawGyro[VEC3_Z];

//			serial_println("%d %d", mpu.rawAccel[0], mpu.rawAccel[1]);

#if 0
			vx = (float)mpu.rawAccel[0] / 100.0f;
			vy = (float)mpu.rawAccel[1] / 100.0f;

			sx += vx / 100.0f;
			sy += vy / 100.0f;

			ctx.sx = sx;
			ctx.sy = sy;
#endif

			if (ctx.nr_point < N_SAMPLE)
				ctx.nr_point++;

			if (count++ % 10 == 0) {
//				serial_println(buf);
				mpu_draw_waveform(&ctx);
			}
		}
	}
}

static int mpu6050_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	vSemaphoreCreateBinary(mpu6050Semaphore);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);

	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
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

//void EXTI9_5_IRQHandler(void)
void EXTI15_10_IRQHandler(void)
//void EXTI1_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;
	uint8_t status;

//	status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
	status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);

	EXTI_ClearITPendingBit(EXTI_Line15);
	if (status == Bit_RESET) { 
		xSemaphoreGiveFromISR(mpu6050Semaphore, &lHigherPriorityTaskWoken );
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}
}
