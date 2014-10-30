#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4xx_adc.h"
#include "stm32f429i_discovery_lcd.h"
#include "GUI.h"
#include "math.h"
#include "string.h"

#define WIDTH 240
#define STEP 10
#define ADC_DMA_BUF_LEN (WIDTH*STEP)

#define RAISING(last, new) (last == LOW && new == HIGH)
#define FALLING(last, new) (last == HIGH && new == LOW)

#define V_RANGE 100
#define V_OFFSET 150

#define BIAS 2048
#define THRESHHOLD 0

#define MAX_BYTE_BUF 128
#define MAX_BIT_BUF (128*8)

enum {
	LOW		= 0,
	HIGH	= 1
};

struct draw_ctx {
	int16_t *data;
	uint16_t nr_point;
	uint32_t freq;
	uint32_t duty;
	uint8_t byte[MAX_BYTE_BUF];
	uint8_t byte_count;
};

SemaphoreHandle_t xPulseSemaphore = NULL;
void ADC1_CH6_DMA_Config(void);
void adc_sample_freq_set(uint32_t freq);
void set_pcm_out_freq(uint32_t freq, uint32_t duty);
void draw_waveform(void *pdata);

static portTASK_FUNCTION_PROTO(vCtrlTask, pvParameters);
static portTASK_FUNCTION_PROTO(vPulseTask, pvParameters);

volatile uint16_t adc_value[ADC_DMA_BUF_LEN];
int32_t sample_freq = 100000;
uint8_t step = STEP;

void vStartCtrlTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vCtrlTask, (signed char *)"Control", 256, NULL, uxPriority, (TaskHandle_t *)NULL);
}

void vStartPulseTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vPulseTask, (signed char *)"Pulse", 1024, NULL, uxPriority, (TaskHandle_t *)NULL);
}

static portTASK_FUNCTION(vCtrlTask, pvParameters)
{
	uint8_t ch, flag;

	while (1) {
		flag = 0;
		serial_read(0, &ch, 1);
		switch (ch) {
			case '=':
				sample_freq += 200;
				flag = 1;
				break;
			case '-':
				sample_freq -= 200;
				flag = 1;
				break;
			case '1':
				sample_freq = 100000;
				flag = 1;
				break;
			case 's':
				ADC_Cmd(ADC1, DISABLE);
				break;
			case 'c':
				ADC_Cmd(ADC1, ENABLE);
				break;
			default:
				serial_println("=:increase\n\r-:decrease\n\r1:reset\n\r");
		}

		if (flag) {
			if (sample_freq < 0)
				sample_freq = 100;
			if (sample_freq > 200000)
				sample_freq = 200000;

			adc_sample_freq_set(sample_freq);
		}
	}
}

static portTASK_FUNCTION(vPulseTask, pvParameters)
{
	uint16_t i, j;
	uint16_t v;
	uint8_t level, last_level;
	int32_t t0, tx, diff, freq, duty, period, tolerance;
	int16_t point[WIDTH];
	GUI_RECT Rect = {0, 60, WIDTH, 300};
	struct draw_ctx ctx = {.data = point, .nr_point = WIDTH, .freq = 0, .duty = 0};
	uint8_t bit[MAX_BIT_BUF], start;
	uint16_t index;

	LCD_Init();
	LTDC_Cmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	GUI_Init();
//	GUI_SelectLayer(1);
//	GUI_SetBkColor(GUI_TRANSPARENT);
	GUI_SetPenSize(1);

	vSemaphoreCreateBinary(xPulseSemaphore);
	xSemaphoreTake(xPulseSemaphore, 0);
	set_pcm_out_freq(10000, 30); /* 2k, 20% duty */
	adc_sample_freq_set(sample_freq);
	ADC1_CH6_DMA_Config();
//	ADC_SoftwareStartConv(ADC1);
	serial_println("start pulse task");

	start = 8;
	index = 0;
	t0 = 0;
	last_level = 0;

	for (;;) {
		xSemaphoreTake(xPulseSemaphore, portMAX_DELAY);
		for (i=0; i<ADC_DMA_BUF_LEN; i++) {
			if (adc_value[i] >= BIAS + THRESHHOLD) {
				level = HIGH;
			} else if (adc_value[i] <= BIAS - THRESHHOLD) {
				level = LOW;
			}
			if (RAISING(last_level, level)) {
				diff = (i - t0) >= 0 ? (i - t0) : (i - t0 + ADC_DMA_BUF_LEN);
				freq = sample_freq / diff;
				if (abs(ctx.freq - freq) > 10)
					ctx.freq = freq;
				t0 = i;
			} else if (FALLING(last_level, level)) {
				diff = i - t0;
				freq = sample_freq / diff;
				duty = ctx.freq * 100 / freq;
				if (abs(ctx.duty - duty) > 5)
					ctx.duty = duty;
			}
			last_level = level;
		}

		for (i=0; i<WIDTH; i++) {
			point[i] = roundf((float)(V_RANGE * adc_value[i*step]) / 4096.0);
		}

		draw_waveform(&ctx);
//		GUI_MEMDEV_Draw(&Rect, &draw_waveform, &ctx, 0, 0);
	}
}

/**
  * @brief  ADC1 channel6 with DMA configuration
  * @param  None
  * @retval None
  */
void ADC1_CH6_DMA_Config(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	DMA_DeInit(DMA2_Stream0);
	/* DMA2 Stream0 channe0 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_value[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_DMA_BUF_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

//	DMA_DoubleBufferModeConfig(DMA2_Stream0, (uint32_t)&adc_value[1], DMA_Memory_0);
//	DMA_DoubleBufferModeCmd(DMA2_Stream0, ENABLE);

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC,ENABLE);              //DMA enable
	DMA_Cmd(DMA2_Stream0, ENABLE);


	/* Configure ADC1 Channel6 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_RisingFalling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channe6 configuration *************************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_3Cycles);

#if 0
	/* Watchdog */
	ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
	ADC_AnalogWatchdogThresholdsConfig(ADC3, 0x0745, 0x026c);  //阈值设置。高：3V 低：1V
	ADC_AnalogWatchdogSingleChannelConfig(ADC3, ADC_Channel_7);
	ADC_ITConfig(ADC3, ADC_IT_AWD, ENABLE);
#endif

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
}

/* duty = x% */
void set_pcm_out_freq(uint32_t freq, uint32_t duty)
{
	serial_println("set pcm out freq");
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* TIM3 channel 8 pin (PC.8) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM pins to PC.8 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);

	TIM_DeInit(TIM3);
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1000000 / freq;
	TIM_TimeBaseStructure.TIM_Prescaler = 90 - 1; // 1M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = duty * 10000/ freq;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

void adc_sample_freq_set(uint32_t freq)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t prescaler, period;

	serial_println("set sample freq to %dhz", freq);
	prescaler = 45 - 1; // 2M
	period = 90000000 / (prescaler + 1) / freq - 1;

	TIM_DeInit(TIM2);
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM IT enable */
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_Pulse = period;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void draw_waveform(void *pdata)
{
	struct draw_ctx *ctx = (struct draw_ctx *)pdata;
	unsigned char buf[64];

	sprintf(buf, "sample freq: %08dhz\nfreq: %06dhz\t\t\tduty:%3d", sample_freq, ctx->freq, ctx->duty);

	GUI_MULTIBUF_Begin();
	GUI_Clear();

	GUI_SetColor(GUI_ORANGE);
	GUI_DrawHLine(V_OFFSET - V_RANGE - 5, 0, WIDTH);
	GUI_DrawHLine(V_OFFSET + V_RANGE + 5, 0, WIDTH);
	if (ctx->byte_count > 0)
		GUI_DispStringAt(ctx->byte, 0, 25);

	GUI_SetLineStyle(GUI_LS_DOT);
	GUI_DrawLine(0, V_OFFSET, WIDTH, V_OFFSET);

	GUI_SetColor(GUI_DARKGREEN);
	GUI_DrawGraph(ctx->data, ctx->nr_point, 0, V_OFFSET);

	GUI_SetTextMode(GUI_TM_TRANS);
	GUI_SetFont(GUI_FONT_8X8);
	GUI_DispStringHCenterAt("Waveform of PB0", 120, 10);

	GUI_SetFont(GUI_FONT_6X8);
	GUI_DispStringAt(buf, 0, 280);

	GUI_SetColor(GUI_RED);
	GUI_DispStringAt("Tips: use (+/-) change sample rate", 0, 310);
	GUI_MULTIBUF_End();
}

void DMA2_Stream0_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;
	uint16_t i, size;
	int x;
	static int count = 0;
	uint8_t buf[16];

	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
		DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
		/*
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);  
		if (DMA_GetCurrentMemoryTarget(DMA2_Stream0) == 0) {
			x = 0;
		} else {
			x = 1;
		}
		*/

		/*
		for (i=0; i<ADC_DMA_BUF_LEN; i++)  {
			size = sprintf(buf, "%04d\n", adc_value[x][i]);
			VCP_send_str(buf, size);
		}
		*/

		if (count % 10000)
			STM_EVAL_LEDToggle(LED3);
		count++;

		xSemaphoreGiveFromISR(xPulseSemaphore, &lHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
//		VCP_send_data(&adc_value, 2);
	}
}

