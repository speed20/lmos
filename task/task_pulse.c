#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4_discovery.h"
#include "stm32f4xx_adc.h"

static portTASK_FUNCTION_PROTO(vPulseTask, pvParameters);
extern uint8_t pulse_start;
extern xSemaphoreHandle xPulseSemaphore;
#define ADC1_DR_Address    ((u32)0x4001204C)
volatile uint16_t ADC_ConvertedValue;
void ADC1_CH6_DMA_Config(void);

void vStartPulseTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vPulseTask, (signed char *)"Pulse", 4096, NULL, uxPriority, (xTaskHandle *)NULL);
}

enum {
	STOP = 0,
	PRE_START,
	START,
};

struct point {
	int32_t t;
	uint32_t v;
};

static portTASK_FUNCTION(vPulseTask, pvParameters)
{
	uint16_t i, count, start_count, index;
	uint16_t AD_value;
	int16_t diff;
	int16_t tmp_data[2], data[16];
	struct point pp[80];
	int tmp;
	int thresh_hold, state;
	uint32_t t_pre, t;
	uint32_t period;
	int32_t dt;
	uint8_t first, byte, bit;

	xSemaphoreTake(xPulseSemaphore, 0);

	ADC1_CH6_DMA_Config();
    ADC_SoftwareStartConv(ADC1);

	while(1) {
		AD_value = ADC_ConvertedValue;
//		serial_print("%04d", AD_value);
//		serial_write(USART_PORT, &AD_value, sizeof(AD_value));
		Delay(1000000);
//		sprintf(data, "%02x%02x", AD_value & 0xff, (AD_value >> 8) & 0xff);
//		VCP_send_str(buf);
	}

#if 0
	period = 0;
	index = count = 0;
	first = 1;
	state = STOP;
	tmp_data[0] = 0;

	while(1) {
		AD_value = ADC_ConvertedValue;
		t = TIM_GetCounter(TIM2);

		if (AD_value > 400)
			tmp_data[1] = 1;
		else
			tmp_data[1] = 0;

		switch (state) {
			case PRE_START:
				{
					diff = tmp_data[1] - tmp_data[0];
#if 0
					if (diff != 0) {
						dt = t- t_pre;
//						dt = t - t_pre >= 0 ? t - t_pre : 65535 + t - t_pre;
						if (dt < 0)
							dt += 0xffff;
						if (diff == 1) {
							pp[count].t = dt;
							pp[count].v = 0;
						} else {
							pp[count].t = dt;
							pp[count].v = 1;
						}
						t_pre = t;
						count++;
					}

					if (count >= 40) goto quit;
#endif

#if 1
					if (diff == 1 && period == 0) { /* raising = 0 */
						if (count == 0)
							t_pre = t;
						else {
							dt = t - t_pre;
							if (dt < 0)
								dt += 0xffff;

							serial_println("dt is %d, freq = %d, count = %d", \
									dt, 8400000 / dt, count);
							period = dt;
							count = 0;
							state = START;
							bit = 0;
							index = 0;
							t_pre = t + period / 2;
							serial_println("start");
						}
						count++;
					}
#endif
				}
				break;
			case START:
				{
					diff = tmp_data[1] - tmp_data[0];
//					serial_println("diff: %d - %d = %d", tmp_data[1], tmp_data[0], diff);
					if (diff != 0) {
						dt = t - t_pre;
						if (dt < 0)
							dt += 0xffff;

						pp[index++].t = dt;
						if (index >= 4) goto quit;

						if (dt > period*1/3) { /* 0 */
							if (diff == 1)
								byte = 1;
							else if (diff == -1)
								byte = 0;
//							serial_println("%d", byte);
							t_pre = t;
						}
					} else {
						dt = t - t_pre;
						if (dt < 0)
							dt += 0xffff;

						if (dt > period * 2) {
							serial_println("start to stop");
							state = STOP;
						}
					}
				}
				break;
			case STOP:
				if (tmp_data[1] == 1)
					state = PRE_START;
				count = 0;
				t_pre = t;
				break;
			default:
				serial_println("bad state");
		}
		tmp_data[0] = tmp_data[1];
	}
#endif

#if 0
	while (1) {
		AD_value = ADC_ConvertedValue;
		if (AD_value >= 400) {
			data[count++] = 100;
		} else {
			data[count++] = 0;
		}
		if (count > 2000)
			break;
		vParTestToggleLED(1);
		Delay(20);
	}
#endif

#if 0
	for (i=0; i<count; i++) {
		serial_println("%d", data[i]);
		vParTestToggleLED(2);
		Delay(10000);
	}

	for (;;) {
		vParTestToggleLED(3);
		Delay(10000);
	}
#endif

quit:
	for (i=0; i<index; i++) {
		serial_println("%d, %d", pp[i].t, pp[i].v);
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
  NVIC_InitTypeDef myNVIC_InitStructure;

	myNVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	myNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	myNVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	myNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&myNVIC_InitStructure);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

//  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  //DMA_ITConfig(DMA2_Stream0, DMA_IT_TC,ENABLE);              //DMA enable
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC1 Channel6 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
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
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC1, ENABLE);
}
