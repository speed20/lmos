#include "FreeRTOS.h"
#include "stm32f4xx_it.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "delay.h"
#include "log.h"
#include "stm32f429i_discovery.h"

#define CURSOR_STEP     7

uint32_t t1, t2;
uint32_t start = 1;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1) {
	vParTestToggleLED(LED3);	
  }
}

void MemManage_Handler(void)
{
  while (1) {
	vParTestToggleLED(LED4);
  }
}

void BusFault_Handler(void)
{
  while (1) {
	vParTestToggleLED(LED3);	
  }
}

void UsageFault_Handler(void)
{
  while (1) {
	vParTestToggleLED(LED3);	
  }
}

void DebugMon_Handler(void)
{
}

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	vParTestToggleLED(LED3);	
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

#if 0
void TIM5_IRQHandler(void)
{
	int32_t tick = 0;
	uint32_t before, after;
	uint32_t val = 0;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	vParTestToggleLED(LED4);	

	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);

	if (start) {
		t2 = TIM_GetCapture2(TIM5);
		start = 0;
		val = 0xBEEF << 16 | GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
	} else {
		t2 = TIM_GetCapture2(TIM5);
		tick = t2 - t1;
		val = tick < 0 ? tick + 0xffff : tick;
	}
	t1 = t2;

	if (xQueueSendFromISR(xIRQueue, &val, &xHigherPriorityTaskWoken) == pdTRUE) {
		if(xHigherPriorityTaskWoken) {
			taskYIELD();
		}
	} else {
		printk("full\r\n");
	}
}

#endif
void EXTI0_IRQHandler(void)
//void EXTI15_10_IRQHandler(void)
{
	printk("button pressed\n");

	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

void Fail_Handler(void)
{
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED4);
  }
}

