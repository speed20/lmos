/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "usb_core.h"
#include "delay.h"
#include "serial.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CURSOR_STEP     7

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t t1, t2;
uint8_t button_flag = 0;
uint8_t update_flag = 0;
uint32_t start = 1;

extern xSemaphoreHandle xTestSemaphore;
extern xSemaphoreHandle mpu6050Semaphore;
extern xSemaphoreHandle xPulseSemaphore;
extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern xQueueHandle xIRQueue;
extern volatile uint16_t adc_value;
#ifdef SERIAL_USE_DMA
extern volatile uint8_t flag_uart_send;
#endif

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
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

	vParTestToggleLED(LED2);	
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

//void EXTI0_IRQHandler(void)

void EXTI15_10_IRQHandler(void)
{
	serial_print("button pressed\r\n");
	long lHigherPriorityTaskWoken = pdFALSE;

	/* Only line 6 is enabled, so there is no need to test which line generated
	the interrupt. */
	EXTI_ClearITPendingBit(USER_BUTTON1_EXTI_LINE);
	
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

void Fail_Handler(void)
{
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED2);
  }
}

//#define ADC_DMA_BUF_LEN 1024
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
			STM_EVAL_LEDToggle(LED1);
		count++;

		xSemaphoreGiveFromISR(xPulseSemaphore, &lHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
//		VCP_send_data(&adc_value, 2);
	}
}

#ifdef SERIAL_USE_DMA
void DMA1_Stream3_IRQHandler(void)  
{  
	if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) {
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);  
		DMA_Cmd(DMA1_Stream3, DISABLE);
		flag_uart_send = 0;  
	}
}
#endif
