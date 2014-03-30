#include "FreeRTOS.h"
#include "stm32f4xx_it.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "delay.h"
#include "serial.h"

#define CURSOR_STEP     7

uint32_t t1, t2;
uint8_t button_flag = 0;
uint8_t update_flag = 0;
uint32_t start = 1;

//extern SemaphoreHandle_t xTestSemaphore;
//extern SemaphoreHandle_t mpu6050Semaphore;
//extern USB_OTG_CORE_HANDLE USB_OTG_dev;
//extern QueueHandle_t xIRQueue;

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

void TIM3_IRQHandler( void )
{
//	serial_println("tim3 irq");
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		delay_flag = 1;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}
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
		serial_print("full\r\n");
	}
}

void EXTI0_IRQHandler(void)
//void EXTI15_10_IRQHandler(void)
{
	serial_print("button pressed\r\n");
	long lHigherPriorityTaskWoken = pdFALSE;

	/* Only line 6 is enabled, so there is no need to test which line generated
	the interrupt. */
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	
	/* This interrupt does nothing more than demonstrate how to synchronise a
	task with an interrupt.  First the handler releases a semaphore.
	lHigherPriorityTaskWoken has been initialised to zero. */
	xSemaphoreGiveFromISR( xTestSemaphore, &lHigherPriorityTaskWoken );
	
	/* If there was a task that was blocked on the semaphore, and giving the
	semaphore caused the task to unblock, and the unblocked task has a priority
	higher than the currently executing task (the task that this interrupt
	interrupted), then lHigherPriorityTaskWoken will have been set to pdTRUE.
	Passing pdTRUE into the following macro call will cause this interrupt to
	return directly to the unblocked, higher priority, task. */
	portEND_SWITCHING_ISR( lHigherPriorityTaskWoken );
}
#endif

/*
void EXTI1_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;
	uint8_t status;

	status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);

	EXTI_ClearITPendingBit(EXTI_Line1);
	if (status == Bit_RESET) { 
		xSemaphoreGiveFromISR(mpu6050Semaphore, &lHigherPriorityTaskWoken );
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}
}
*/

#if 0
/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{   
  if(USB_OTG_dev.cfg.low_power)
  {   
    /* Reset SLEEPDEEP and SLEEPONEXIT bits */
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

    /* After wake-up from sleep mode, reconfigure the system clock */
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}   
    
/**   
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{   
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
#endif

void Fail_Handler(void)
{
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED4);
  }
}

