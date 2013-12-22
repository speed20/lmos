#include "stm32f4xx.h"

#define LD3320_SPI                       SPI2
#define LD3320_SPI_CLK                   RCC_APB1Periph_SPI2

#define LD3320_SPI_SCK_PIN               GPIO_Pin_13                  /* PB.13 */
#define LD3320_SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOB */
#define LD3320_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define LD3320_SPI_SCK_SOURCE            GPIO_PinSource13
#define LD3320_SPI_SCK_AF                GPIO_AF_SPI2

#define LD3320_SPI_MISO_PIN              GPIO_Pin_14                  /* PB.14 */
#define LD3320_SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOB */
#define LD3320_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define LD3320_SPI_MISO_SOURCE           GPIO_PinSource14
#define LD3320_SPI_MISO_AF               GPIO_AF_SPI2

#define LD3320_SPI_MOSI_PIN              GPIO_Pin_15                  /* PB.15 */
#define LD3320_SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOB */
#define LD3320_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define LD3320_SPI_MOSI_SOURCE           GPIO_PinSource15
#define LD3320_SPI_MOSI_AF               GPIO_AF_SPI2

#define LD3320_SPI_CS_PIN                GPIO_Pin_2                  /* PA.8 */
#define LD3320_SPI_CS_GPIO_PORT          GPIOC                       /* GPIOA */
#define LD3320_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define LD3320_DC_PIN					 GPIO_Pin_2                  /* PB.2 */
#define LD3320_DC_GPIO_PORT			   	 GPIOB                       /* GPIOB */
#define LD3320_DC_GPIO_CLK			   	 RCC_AHB1Periph_GPIOB

#define LD3320_RESET_PIN				 GPIO_Pin_1                  /* PA.3 */
#define LD3320_RESET_GPIO_PORT			 GPIOC                       /* GPIOA */
#define LD3320_RESET_GPIO_CLK			 RCC_AHB1Periph_GPIOC

#define LD3320_CS_LOW()					 GPIO_ResetBits(LD3320_SPI_CS_GPIO_PORT, LD3320_SPI_CS_PIN)
#define LD3320_CS_HIGH()				 GPIO_SetBits(LD3320_SPI_CS_GPIO_PORT, LD3320_SPI_CS_PIN)

#define LD3320_DC_LOW()					 GPIO_ResetBits(LD3320_DC_GPIO_PORT, LD3320_DC_PIN)
#define LD3320_DC_HIGH()				 GPIO_SetBits(LD3320_DC_GPIO_PORT, LD3320_DC_PIN)

#define LD3320_RESET_LOW()				 GPIO_ResetBits(LD3320_RESET_GPIO_PORT, LD3320_RESET_PIN)
#define LD3320_RESET_HIGH()				 GPIO_SetBits(LD3320_RESET_GPIO_PORT, LD3320_RESET_PIN)

#define LD3320_IRQ_PIN					GPIO_Pin_9
#define LD3320_IRQ_CLK					RCC_AHB1Periph_GPIOD
#define LD3320_IRQ_PORT					GPIOD
#define LD3320_EXTI_PORT				EXTI_PortSourceGPIOD
#define LD3320_EXTI_PIN_SOURCE			EXTI_PinSource9
#define LD3320_IRQ_LINE					EXTI_Line9
#define LD3320_IRQn						EXTI9_5_IRQn

#define CLK_IN   		    22.1184	//  用户可以根据提供给LD3320模块的实际晶振频率自行修改
#define LD_PLL_11			(uint8_t)((CLK_IN/2.0)-1)

#define LD_PLL_ASR_19 		(uint8_t)(CLK_IN*32.0/(LD_PLL_11+1) - 0.51)
#define LD_PLL_ASR_1B 		0x48
#define LD_PLL_ASR_1D 		0x1f

#define LD_PLL_MP3_19       0x0f
#define LD_PLL_MP3_1B       0x18
#define LD_PLL_MP3_1D       (uint8_t)(((90.0*((LD_PLL_11)+1))/(CLK_IN))-1)

#define MASK_INT_SYNC				0x10
#define MASK_INT_FIFO				0x04
#define MASK_AFIFO_INT				0x01
#define MASK_FIFO_STATUS_AFULL		0x08
#define PLAY_END						0x20

#define MIC_VOL     0x4C  //调节麦克风的灵敏度，数值越大，反应越灵敏，
#define SPEAKER_VOL 0x01
#define ITEM_COUNT  4	//定义识别语音个数	  
#define VOICE_COUNT 10

//	以下五个状态定义用来记录程序是在运行ASR识别过程中的哪个状态
enum {
	LD_ASR_NONE = 0x00,	//	表示没有在作ASR识别
	LD_ASR_RUNING =	0x01,	//	表示LD3320正在作ASR识别中
	LD_ASR_FOUNDOK = 0x10,	//	表示一次识别流程结束后，有一个识别结果
	LD_ASR_FOUNDZERO = 0x11,	//	表示一次识别流程结束后，没有识别结果
	LD_ASR_ERROR = 0x31	//	表示一次识别流程中LD3320芯片内部出现不正确的状态
};

enum LD_MODE {
	LD_MODE_IDLE = 0x00,
	LD_MODE_ASR = 0x08,
	LD_MODE_MP3 = 0x40
};

extern xSemaphoreHandle xASRSemaphore;
extern xSemaphoreHandle xMP3Semaphore;
extern volatile uint8_t nAsrStatus;
extern volatile uint8_t nLD_Mode;
volatile extern uint32_t mp3_cur, mp3_size;
volatile extern uint8_t *mp3_data;

#ifndef NULL
#define NULL 0 
#endif
