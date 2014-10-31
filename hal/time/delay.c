#include "delay.h"
#include "stm32f4xx_tim.h"

volatile uint8_t delay_flag = 0;

void Delay(uint32_t usec)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	uint32_t period, prescaler;

	delay_flag = 0;

	if (usec < 65536) { /* precision 1us, 0~65536us */
		period = usec;
		prescaler = 180;
	} else if (usec < 655360) { /* precision 0.01ms, 0~0.65536s */
		period = usec / 10;
		prescaler = 1800;
	} else if (usec < 6553600) { /* precision 0.1ms, 0~6.5536s */
		period = usec / 100;
		prescaler = 18000;
	} else {
		printk("usec overflow\n");
		return ;
	}

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure );

	TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = period - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	while (delay_flag == 0) ;

	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		delay_flag = 1;
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
