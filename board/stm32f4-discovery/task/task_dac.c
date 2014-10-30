/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "pin_map.h"
#include "stm32f4xx_dac.h"

static portTASK_FUNCTION_PROTO(vDACTask, pvParameters);
static TaskHandle_t dac_handle;

void DAC_Config(void);
void vStartDACTask(unsigned portBASE_TYPE uxPriority)
{
	xTaskCreate(vDACTask, ( signed char * )"DAC", 1024, NULL, uxPriority, (TaskHandle_t *)&dac_handle);
}

static portTASK_FUNCTION(vDACTask, pvParameters)
{
	DAC_Config();
	int vol = 0;

	for (;;) {
		DAC_SetChannel2Data(DAC_Align_12b_R, vol);    //设置数据右对齐，转换值设置为2047
		DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);    //软件触发通道2 开始转换
		Delay(10);
		vol += 10;
	}
}

struct pin_map dac_pin_map[2] = {
{GPIO_Pin_4, GPIO_PinSource4, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, -1, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AIN},
{GPIO_Pin_5, GPIO_PinSource5, GPIOA, AHB1, RCC_AHB1Periph_GPIOA, -1, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_Mode_AIN}
};

void DAC_Config(void)
{
    DAC_InitTypeDef DAC_InitStructure;
	int i;

	for (i=1; i<2; i++) {
		io_request(&dac_pin_map[i]);
	}

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;       //设置触发方式为软件触发
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;     //无波形产生
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;    //DAC输出缓冲使能
//    DAC_Init(DAC_Channel_1 ,&DAC_InitStructure); //使能以上设置到DAC1               
    DAC_Init(DAC_Channel_2 ,&DAC_InitStructure); //使能以上设置到DAC2
 
 //   DAC_Cmd(DAC_Channel_1 ,ENABLE);      //最后的开关
    DAC_Cmd(DAC_Channel_2 ,ENABLE);    //最后的开关


//    DAC_SetChannel1Data(DAC_Align_12b_R,1028);    //设置数据右对齐，转换值设置为1028
 //   DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);    //软件触发通道1 开始转换
 
	/*
    DAC_SetChannel2Data(DAC_Align_12b_R,2047);    //设置数据右对齐，转换值设置为2047
    DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);    //软件触发通道2 开始转换
	*/
}
