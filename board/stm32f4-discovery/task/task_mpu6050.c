#include <stdio.h>
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "usb_core.h"
#include <invense/mpu6050.h>
#include <mpulib/mpulib.h>
#include "GUI.h"

#define STACK_SIZE 1024

SemaphoreHandle_t mpu6050Semaphore;
static portTASK_FUNCTION_PROTO(vMPUTask, pvParameters);
static int mpu6050_config();

void vStartMPUTasks(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vMPUTask, (signed char *)"imu", STACK_SIZE, NULL, uxPriority, (TaskHandle_t *)NULL);
}

struct draw_ctx {
	int16_t *data_roll;
	int16_t *data_yaw;
	int16_t *data_pitch;
	uint16_t nr_point;
};

#define V_OFFSET 160

void mpu_draw_waveform(void *pdata)
{
	struct draw_ctx *ctx = (struct draw_ctx *)pdata;
	unsigned char buf[64];

	sprintf(buf, "%04d, %04d, %04d", ctx->data_roll[0], ctx->data_yaw[0], ctx->data_pitch[0]);
	serial_println("%s", buf);

#if 1
	GUI_MULTIBUF_Begin();
	GUI_Clear();

//	GUI_DispStringHCenterAt(buf, 120, 25);

	GUI_SetColor(GUI_RED);
	GUI_DrawGraph(ctx->data_roll, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_DARKGREEN);
	GUI_DrawGraph(ctx->data_yaw, ctx->nr_point, 0, V_OFFSET);

	GUI_SetColor(GUI_BLUE);
	GUI_DrawGraph(ctx->data_pitch, ctx->nr_point, 0, V_OFFSET);
#endif

	GUI_MULTIBUF_End();
}

#define SCALE 2000
#define N_SAMPLE 240
static portTASK_FUNCTION(vMPUTask, pvParameters)
{
	unsigned char buf[128];
	uint32_t size, i;
	mpudata_t mpu;
	int16_t roll[N_SAMPLE], yaw[N_SAMPLE], pitch[N_SAMPLE];
	struct draw_ctx ctx = {roll, yaw, pitch, 1};
	uint32_t count;

	//memset(&mpu, 0, sizeof(mpu));
	mpu6050_config();

#if 1
	LCD_Init();
	LTDC_Cmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	GUI_Init();
//	GUI_SelectLayer(1);
	GUI_SetBkColor(GUI_TRANSPARENT);
	GUI_SetPenSize(1);
#endif

	xSemaphoreTake(mpu6050Semaphore, 0);

	if (mpulib_init(100, 10) < 0) {
		serial_println("mpulib init failed");
		for (;;);
	}
	serial_println("mpu config finish");

//	set_calibration(0);
//	set_calibration(1);

	count = 0;
	for (;;) {
		xSemaphoreTake(mpu6050Semaphore, portMAX_DELAY);
		if (mpulib_read(&mpu) == 0) {
			size = sprintf(buf, "%03d,%03d,%03d", (int32_t)((mpu.fusedEuler[VEC3_X]) * RAD_TO_DEGREE), \
					(int32_t)((mpu.fusedEuler[VEC3_Y]) * RAD_TO_DEGREE), \
					(int32_t)((mpu.fusedEuler[VEC3_Z]) * RAD_TO_DEGREE));
//			serial_println("%s", buf);

			for (i=ctx.nr_point-1; i>0; i--) {
				roll[i] = roll[i-1];
				yaw[i] = yaw[i-1];
				pitch[i] = pitch[i-1];
			}

			roll[0] = mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE * 120.0 / 180.0;
			yaw[0] = mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE * 120.0 / 180.0;
			pitch[0] = mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE * 120.0 / 180.0;

			if (ctx.nr_point < N_SAMPLE)
				ctx.nr_point++;

			if (count++ % 10 == 0)
				mpu_draw_waveform(&ctx);
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
