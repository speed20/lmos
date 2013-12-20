#include "FreeRTOS.h"
#include "semphr.h"
#include "ld3320.h"

/* PA5: clock
 * PA6: MISO
 * PA7: MOSI
 * PA8: CS
 * PA9: IRQ
 */

xSemaphoreHandle xASRSemaphore = NULL;
xSemaphoreHandle xMP3Semaphore = NULL;
volatile uint8_t nAsrStatus;
volatile uint8_t nLD_Mode;
volatile uint32_t mp3_start, mp3_cur, mp3_size;
volatile uint8_t *mp3_data;

void ld3320_io_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable the SPI periph */
	RCC_APB1PeriphClockCmd(LD3320_SPI_CLK, ENABLE);

	/* Enable SCK, MOSI clocks */
	RCC_AHB1PeriphClockCmd(LD3320_SPI_SCK_GPIO_CLK | LD3320_SPI_MOSI_GPIO_CLK | LD3320_SPI_MISO_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(LD3320_SPI_SCK_GPIO_PORT, LD3320_SPI_SCK_SOURCE, LD3320_SPI_SCK_AF);
	GPIO_PinAFConfig(LD3320_SPI_MISO_GPIO_PORT, LD3320_SPI_MISO_SOURCE, LD3320_SPI_MISO_AF);
	GPIO_PinAFConfig(LD3320_SPI_MOSI_GPIO_PORT, LD3320_SPI_MOSI_SOURCE, LD3320_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = LD3320_SPI_SCK_PIN;
	GPIO_Init(LD3320_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  LD3320_SPI_MISO_PIN;
	GPIO_Init(LD3320_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  LD3320_SPI_MOSI_PIN;
	GPIO_Init(LD3320_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(LD3320_SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_SSOutputCmd(LD3320_SPI, ENABLE);
	SPI_Init(LD3320_SPI, &SPI_InitStructure);

	/* Enable SPI2  */
	SPI_Cmd(LD3320_SPI, ENABLE);

	/* Enable CS  GPIO clock */
	RCC_AHB1PeriphClockCmd(LD3320_SPI_CS_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LD3320_DC_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LD3320_RESET_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LD3320_IRQ_CLK, ENABLE);

	/* Configure GPIO PIN for Chip select */
	GPIO_InitStructure.GPIO_Pin = LD3320_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(LD3320_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Deselect : Chip Select low */
	LD3320_CS_HIGH();

	/* DC PIN, PE12 */
	GPIO_InitStructure.GPIO_Pin = LD3320_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LD3320_DC_GPIO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(LD3320_DC_GPIO_PORT, LD3320_DC_PIN);

	/* RESET PIN, PA3 */
	GPIO_InitStructure.GPIO_Pin = LD3320_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LD3320_RESET_GPIO_PORT, &GPIO_InitStructure);

#if 1
	/* IRQ */
	GPIO_InitStructure.GPIO_Pin = LD3320_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LD3320_IRQ_PORT, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(LD3320_EXTI_PORT, LD3320_EXTI_PIN_SOURCE);

	/* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = LD3320_IRQ_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = LD3320_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void ld3320_reset()
{
	LD3320_RESET_HIGH();
	Delay(5000);
	LD3320_RESET_LOW();
	Delay(5000);
	LD3320_RESET_HIGH();
}

uint8_t spi_send_recv_byte(uint8_t byte)
{
	uint32_t timeout;
	uint8_t val;

	timeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(LD3320_SPI, SPI_I2S_FLAG_TXE) == RESET) {
		if(timeout-- == 0) {
			serial_println("spi send timeout");
			break;
		}
	}

	SPI_I2S_SendData(LD3320_SPI, byte);

	timeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(LD3320_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
		if(timeout-- == 0) {
			serial_println("spi receive timeout");
			break;
		}
	}

	val = (uint8_t)SPI_I2S_ReceiveData(LD3320_SPI);

	return val;
}

void ld3320_write_reg(uint8_t reg, uint8_t val)
{
	LD3320_CS_LOW();

	spi_send_recv_byte(0x04);
	spi_send_recv_byte(reg);
	spi_send_recv_byte(val);

	LD3320_CS_HIGH();
}

uint8_t ld3320_read_reg(uint8_t reg)
{
	uint8_t val;

	LD3320_CS_LOW();

	spi_send_recv_byte(0x05);
	spi_send_recv_byte(reg);
	val = spi_send_recv_byte(0x00);

	LD3320_CS_HIGH();

	return val;
}

void ld3320_init_common()
{ 
	ld3320_read_reg(0x06);  
	ld3320_write_reg(0x17, 0x35); 
	Delay(10000);
	ld3320_read_reg(0x06);  

	ld3320_write_reg(0x89, 0x03);  
	Delay(50000);
	ld3320_write_reg(0xCF, 0x43);   
	Delay(50000);
	ld3320_write_reg(0xCB, 0x02);
	
	/*PLL setting*/
	ld3320_write_reg(0x11, LD_PLL_11);       
	
	ld3320_write_reg(0x1E,0x00);
	ld3320_write_reg(0x19, LD_PLL_ASR_19); 
	ld3320_write_reg(0x1B, LD_PLL_ASR_1B);		
	ld3320_write_reg(0x1D, LD_PLL_ASR_1D);

	Delay(10000);
	
	ld3320_write_reg(0xCD, 0x04);
	ld3320_write_reg(0x17, 0x4C); 
	Delay(50000);
	ld3320_write_reg(0xB8, 0x05); /* timeout 5sec */
	ld3320_write_reg(0xB9, 0x00);
	ld3320_write_reg(0xCF, 0x4F); 
	ld3320_write_reg(0x6F, 0xFF); 
} 

void ld3320_init_ASR()
{	
	ld3320_init_common();

	ld3320_write_reg(0xBD, 0x00);
	ld3320_write_reg(0x17, 0x48);
	Delay(10000);

	ld3320_write_reg(0x3C, 0x80);    
	ld3320_write_reg(0x3E, 0x07);
	ld3320_write_reg(0x38, 0xff);    
	ld3320_write_reg(0x3A, 0x07);
	
	ld3320_write_reg(0x40, 0);          
	ld3320_write_reg(0x42, 8);
	ld3320_write_reg(0x44, 0);    
	ld3320_write_reg(0x46, 8); 
	Delay(1000);
}

// Return 1: success.
uint8_t ld3320_Check_ASRBusyFlag_b2()
{
	uint8_t j;
	uint8_t flag = 0;

	for (j=0; j<10; j++) {
		if (ld3320_read_reg(0xb2) == 0x21) {
			flag = 1;
			break;
		}
		Delay(10000);		
	}
	return flag;
}

void ld3320_AsrStart()
{
	nLD_Mode = LD_MODE_ASR;
	ld3320_init_ASR();
}

// Return 1: success.
uint8_t ld3320_AsrRun()
{
	ld3320_write_reg(0x35, MIC_VOL);
    ld3320_write_reg(0xB3, 0x05);	// 用户阅读 开发手册 理解B3寄存器的调整对于灵敏度和识别距离的影响	
							    // 配合MIC，越大越灵敏
	ld3320_write_reg(0x1C, 0x09);
	ld3320_write_reg(0xBD, 0x20);
	ld3320_write_reg(0x08, 0x01);
	Delay(1000);
	ld3320_write_reg(0x08, 0x00);
	Delay(1000 );

	if (ld3320_Check_ASRBusyFlag_b2() == 0)
		return 0;

//			for (;;);

	ld3320_write_reg(0xB2, 0xff);	
	ld3320_write_reg(0x37, 0x06);
	Delay(5000);
	ld3320_write_reg(0x1C, 0x0b);
	ld3320_write_reg(0x29, 0x10);

//	ld3320_write_reg(0xBD, 0x00);
	return 1;
}

void ld3320_AsrAddFixed_ByString(char * pRecogString, uint8_t k)
{
	uint8_t nAsrAddLength;

	if (*pRecogString == 0)
		return;

	ld3320_write_reg(0xc1, k );
	ld3320_write_reg(0xc3, 0 );
	ld3320_write_reg(0x08, 0x04);
	Delay(1000);
	ld3320_write_reg(0x08, 0x00);
	Delay(1000);	

	for (nAsrAddLength=0; nAsrAddLength<50; nAsrAddLength++) {
		if (pRecogString[nAsrAddLength] == 0)
			break;
		ld3320_write_reg(0x5, pRecogString[nAsrAddLength]);
	}
	
	ld3320_write_reg(0xb9, nAsrAddLength);
	ld3320_write_reg(0xb2, 0xff);
	ld3320_write_reg(0x37, 0x04);
}

unsigned char * str_pattern[] = {
	"bei jing",   //北京
	"shang hai",   //上海
	"tian jin",   //天津
	"chong qing",   //重庆
	"guang zhou",   //广州
	"hang zhou",   //杭州
	"cheng du",   //成都
	"nan jing",   //南京
	"ni hao", 	  //你好   
	"guan deng",    //关灯，两个等同时关闭
	"tai deng kai",  //台灯开启，JDQ01
	"tai deng guan", //台灯关闭，JDQ01
	"bi  deng kai",  //壁灯开启，JDQ02
	"bi  deng guan", //壁灯关闭，JDQ02
	"kai deng",
	"wa sai", 	   //哇塞
};

void ld3320_AsrAddFixed_ByIndex(uint8_t nIndex)
{
	ld3320_AsrAddFixed_ByString(str_pattern[nIndex], nIndex);
}

// Return 1: success.
//	添加识别关键词语，开发者可以学习"语音识别芯片LD3320高阶秘籍.pdf"中关于垃圾词语吸收错误的用法
uint8_t ld3320_AsrAddFixed()
{
	uint8_t k, flag; 			

	flag = 1;
	for (k=0; k<ITEM_COUNT; k++) {	 			
		if(ld3320_Check_ASRBusyFlag_b2() == 0) {
			flag = 0;
			break;
		}		
		ld3320_AsrAddFixed_ByIndex(k);
	}
    return flag;
}	   

uint8_t ld3320_GetResult()
{
	return ld3320_read_reg(0xc5);	  	
}

void ld3320_init_mp3();
void play_sound(uint8_t *data, uint32_t size);
void ld3320_set_vol(uint8_t vol);
void ld3320_play();
void ld3320_load_mp3_data();

void play_sound(uint8_t *data, uint32_t size)
{
	mp3_cur = 0;
	mp3_size = size;
	mp3_data = data;

	ld3320_init_mp3();
	ld3320_set_vol(SPEAKER_VOL);
	ld3320_play();
}

void ld3320_init_mp3()
{
	nLD_Mode = LD_MODE_MP3;
	ld3320_init_common();

	ld3320_write_reg(0xBD,0x02);
	ld3320_write_reg(0x17, 0x48);
	Delay(10000);

	ld3320_write_reg(0x85, 0x52); 
	ld3320_write_reg(0x8F, 0x00);  
	ld3320_write_reg(0x81, 0x00);
	ld3320_write_reg(0x83, 0x00);
	ld3320_write_reg(0x8E, 0xff);
	ld3320_write_reg(0x8D, 0xff);
    Delay(1000);
	ld3320_write_reg(0x87, 0xff);
	ld3320_write_reg(0x89, 0xff);
	Delay(1000);
	ld3320_write_reg(0x22, 0x00);    
	ld3320_write_reg(0x23, 0x00);
	ld3320_write_reg(0x20, 0xef);    
	ld3320_write_reg(0x21, 0x07);
	ld3320_write_reg(0x24, 0x77);          
    ld3320_write_reg(0x25, 0x03);
    ld3320_write_reg(0x26, 0xbb);    
    ld3320_write_reg(0x27, 0x01); 
}
void ld3320_set_vol(uint8_t vol)
{
	vol = ((15 - vol) & 0x0f) << 2;
	ld3320_write_reg(0x8E, vol | 0xc3); 
	ld3320_write_reg(0x87, 0x78); 
}

void ld3320_play()
{
	if (mp3_cur >= mp3_size) {
		serial_println("succed size");
		return ; 
	}

	ld3320_load_mp3_data();

    ld3320_write_reg(0xBA, 0x00);
	ld3320_write_reg(0x17, 0x48);
	ld3320_write_reg(0x33, 0x01);
	ld3320_write_reg(0x29, 0x04);
	
	ld3320_write_reg(0x02, 0x01); 
	ld3320_write_reg(0x85, 0x5A);
}

void ld3320_load_mp3_data()
{
	serial_println("load");
	uint8_t val;
	uint8_t ucStatus;

	ucStatus = ld3320_read_reg(0x06);
	while (!(ucStatus & MASK_FIFO_STATUS_AFULL) && (mp3_cur < mp3_size)) {
		val = *(mp3_data + mp3_cur);
		mp3_cur++;
		ld3320_write_reg(0x01,val);
		ucStatus = ld3320_read_reg(0x06);
	}
}

uint8_t RunASR()
{
	uint8_t i, asrflag = 0;

	for (i=0; i<5; i++) {			//	防止由于硬件原因导致LD3320芯片工作不正常，所以一共尝试5次启动ASR识别流程
		ld3320_AsrStart();
		Delay(1000);

		if (ld3320_AsrAddFixed() == 0) {
			ld3320_reset();			//	LD3320芯片内部出现不正常，立即重启LD3320芯片
			Delay(10000);			//	并从初始化开始重新ASR识别流程
			continue;
		}

		Delay(10000);
		if (ld3320_AsrRun() == 0) {
			ld3320_reset();			//	LD3320芯片内部出现不正常，立即重启LD3320芯片
			Delay(1000);			//	并从初始化开始重新ASR识别流程
			continue;
		}
		asrflag=1;
		break;					//	ASR流程启动成功，退出当前for循环。开始等待LD3320送出的中断信号
	}
	return asrflag;
}

int ld3320_init()
{
	uint8_t v1, v2, v3;

	ld3320_io_init();
	ld3320_reset();

	serial_println("reg: 0x%02x", ld3320_read_reg(0x06));
	serial_println("reg: 0x%02x", ld3320_read_reg(0x06));
	serial_println("reg: 0x%02x", ld3320_read_reg(0x35));
	serial_println("reg: 0x%02x", ld3320_read_reg(0xb3));

	vSemaphoreCreateBinary(xASRSemaphore);
	vSemaphoreCreateBinary(xMP3Semaphore);
}

int ld3320_test()
{
	uint8_t v1, v2, v3;

#if 1
	ld3320_read_reg(0x6);
	ld3320_write_reg(0x35, 0x33);
	ld3320_write_reg(0x1b, 0x55);
	ld3320_write_reg(0xb3, 0xaa);

	v1 = ld3320_read_reg(0x35);
	v2 = ld3320_read_reg(0x1b);
	v3 = ld3320_read_reg(0xb3);
#endif

//	serial_println("0x33 = 0x%02x 0x55 = 0x%02x 0xaa = 0x%02x\n", v1, v2, v3);

	return v1 == 0x33 && v2 == 0x55 && v3 == 0xaa;
}

void EXTI9_5_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;
	uint8_t nAsrResCount = 0;
	uint8_t ucRegVal, ucHighInt, ucLowInt;
	uint8_t status;

	if (nAsrStatus == LD_ASR_RUNING && nLD_Mode == LD_MODE_ASR) {
		if (status == Bit_RESET) {
			ucRegVal = ld3320_read_reg(0x2B);
			ucHighInt = ld3320_read_reg(0x29) ;
			ld3320_write_reg(0x29,0) ;

			if ((ucRegVal & 0x10)
					&& ld3320_read_reg(0xb2) == 0x21
					&& ld3320_read_reg(0xbf) == 0x35) {

				nAsrResCount = ld3320_read_reg(0xba);
			
				if(nAsrResCount > 0 && nAsrResCount < 4) {
					nAsrStatus = LD_ASR_FOUNDOK;
				} else {
					nAsrStatus = LD_ASR_FOUNDZERO;
				}	
			} else {
				nAsrStatus = LD_ASR_FOUNDZERO;
			}
				
			ld3320_write_reg(0x2b, 0);
			ld3320_write_reg(0x1C, 0);
			
			xSemaphoreGiveFromISR(xASRSemaphore, &lHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
		}
	}

	// 声音播放产生的中断，有三种：
	// A. 声音数据已全部播放完。
	// B. 声音数据已发送完毕。
	// C. 声音数据暂时将要用完，需要放入新的数据。	
	if (nLD_Mode == LD_MODE_MP3) {
		ucHighInt = ld3320_read_reg(0x29); 
		ucLowInt = ld3320_read_reg(0x02); 
		ld3320_write_reg(0x29,0) ;
		ld3320_write_reg(0x02,0) ;

		if (ld3320_read_reg(0xBA) & MP3_END) {
			serial_println("play done");
			ld3320_write_reg(0x2B, 0);
			ld3320_write_reg(0xBA, 0);	
			ld3320_write_reg(0xBC,0x0);	
			ld3320_write_reg(0x08,1);
//			Delay(5000);
			ld3320_write_reg(0x08,0);
			ld3320_write_reg(0x33, 0);	 

			xSemaphoreGiveFromISR(xMP3Semaphore, &lHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);

		} else if(mp3_cur >= mp3_size) {
			serial_println("send done");
			ld3320_write_reg(0xBC, 0x01);
			ld3320_write_reg(0x29, 0x10);
		} else {
			serial_println("need load");
			ld3320_load_mp3_data();
			ld3320_write_reg(0x29, ucHighInt); 
			ld3320_write_reg(0x02, ucLowInt) ;
//			Delay(10000);
		}
	}

	status = GPIO_ReadInputDataBit(LD3320_IRQ_PORT, LD3320_IRQ_PIN);
	EXTI_ClearITPendingBit(LD3320_IRQ_LINE);
}
