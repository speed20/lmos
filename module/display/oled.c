#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  
#include "delay.h"

/* OLED GMEM
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127 
*/
uint8_t OLED_GRAM[128][8];

enum {
	OLED_CMD = 0,
	OLED_DATA = 1,
};

/* pin define:
 * PE11: CS
 * PE12: D/C
 * PE13: RST
 */
void oled_io_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

#if 1
	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(OLED_SPI_CLK, ENABLE);

	/* Enable SCK, MOSI clocks */
	RCC_AHB1PeriphClockCmd(OLED_SPI_SCK_GPIO_CLK | OLED_SPI_MOSI_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(OLED_SPI_SCK_GPIO_PORT, OLED_SPI_SCK_SOURCE, OLED_SPI_SCK_AF);
	GPIO_PinAFConfig(OLED_SPI_MOSI_GPIO_PORT, OLED_SPI_MOSI_SOURCE, OLED_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = OLED_SPI_SCK_PIN;
	GPIO_Init(OLED_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  OLED_SPI_MOSI_PIN;
	GPIO_Init(OLED_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(OLED_SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_SSOutputCmd(OLED_SPI, ENABLE);
	SPI_Init(OLED_SPI, &SPI_InitStructure);

	/* Enable SPI1  */
	SPI_Cmd(OLED_SPI, ENABLE);
#endif

	/* Enable CS  GPIO clock */
	RCC_AHB1PeriphClockCmd(OLED_SPI_CS_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(OLED_DC_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(OLED_RST_GPIO_CLK, ENABLE);

	/* Configure GPIO PIN for Chip select */
	/* PE11 */
	GPIO_InitStructure.GPIO_Pin = OLED_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(OLED_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Deselect : Chip Select low */
	OLED_CS_HIGH();

	/* DC PIN, PE12 */
	GPIO_InitStructure.GPIO_Pin = OLED_DC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OLED_DC_GPIO_PORT, &GPIO_InitStructure);

	/* RESET PIN, PE13 */
	GPIO_InitStructure.GPIO_Pin = OLED_RST_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OLED_RST_GPIO_PORT, &GPIO_InitStructure);

	OLED_RST_LOW();
}

void oled_write_byte(uint8_t data, uint8_t cmd)
{
	uint32_t timeout;

	if (cmd == OLED_CMD)
		OLED_DC_LOW();
	else
		OLED_DC_HIGH();

	OLED_CS_LOW();

	timeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(OLED_SPI, SPI_I2S_FLAG_TXE) == RESET) {
		if((timeout --) == 0) {
			printk("lcd write timeout");
			break;
		}
	}

	SPI_I2S_SendData(OLED_SPI, data);

	timeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(OLED_SPI, SPI_I2S_FLAG_BSY) != RESET) {
		if((timeout --) == 0) {
			printk("lcd write timeout");
			break;
		}
	}

	OLED_CS_HIGH();

//	Delay(100000);

#if 0
	timeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(OLED_SPI, SPI_I2S_FLAG_BSY) != RESET) {
//		printk("busy");
		if((timeout--) == 0) {
			printk("lcd write timeout");
			break;
		}
	}
#endif

}

void oled_update_gram(void)
{
	uint8_t i,j;

	for(i=0; i<8; i++)
	{
		oled_write_byte(0xb0 + i, OLED_CMD);    //set page address（0~7）
		oled_write_byte(0x00, OLED_CMD);      //set display pos LSB
		oled_write_byte(0x10, OLED_CMD);      //set display pos MSB

		for(j=0; j<128; j++)
			oled_write_byte(OLED_GRAM[j][i], OLED_DATA);
	}
}

void oled_display_on(void)
{
	oled_write_byte(0X8D, OLED_CMD);  //SET DCDC
	oled_write_byte(0X14, OLED_CMD);  //DCDC ON
	oled_write_byte(0XAF, OLED_CMD);  //DISPLAY ON
}

void old_display_off(void)
{
	oled_write_byte(0X8D, OLED_CMD);  //SET DCDC
	oled_write_byte(0X10, OLED_CMD);  //DCDC OFF
	oled_write_byte(0XAE, OLED_CMD);  //DISPLAY OFF
}		   

void oled_clear(void)
{
	uint8_t i,j;

	for(i=0; i<8; i++)
		for(j=0; j<128; j++)
			OLED_GRAM[j][i] = 0x00;

	oled_update_gram();
}

//x:0~127
//y:0~63
//t:1 set 0, clear
void oled_draw_point(uint8_t x, uint8_t y, uint8_t t)
{
	uint8_t pos,bx,tmp;

	if(x>127 || y>63)
		return; /*out of range.*/

	pos = 7 - y/8;
	bx = y % 8;
	tmp = 1 << (7-bx);

	if (t)
		OLED_GRAM[x][pos] |= tmp;
	else
		OLED_GRAM[x][pos] &= ~tmp;
}

//x1,y1,x2,y2 填充区域的对角坐标
//确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 
//dot:0,清空;1,填充
void oled_fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2, uint8_t dot)
{
	uint8_t x,y;
	for(x=x1; x<=x2; x++) {
		for(y=y1; y<=y2; y++)
			oled_draw_point(x, y, dot);
	}
	oled_update_gram();
}

//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void oled_show_char(uint8_t x, uint8_t y, uint8_t ch, uint8_t size, uint8_t mode)
{
	uint8_t tmp, t, t1;
	uint8_t y0;

	y0 = y;
	ch = ch - ' ';
    for (t=0; t < size; t++) {
		if (size == 12)
			tmp = oled_asc2_1206[ch][t];
		else
			tmp = oled_asc2_1608[ch][t];

        for (t1=0; t1 < 8; t1++) {
			if (tmp & 0x80)
				oled_draw_point(x, y, mode);
			else
				oled_draw_point(x, y, !mode);
			tmp <<= 1;
			y++;
			if ((y-y0) == size) {
				y = y0;
				x++;
				break;
			}
		}
    }
}

/* show string
 * x, y: start point
 * p: pointer to string
 */
//用16字体
void oled_show_string(uint8_t x, uint8_t y, const uint8_t *p)
{
	uint8_t i, j;
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58

	for(i=0; i<8; i++)
		for(j=0; j<128; j++)
			OLED_GRAM[j][i] = 0x00;

    while(*p != '\0') {
		if (x > MAX_CHAR_POSX) {
			x = 0;
			y += 16;
		}

        if (y > MAX_CHAR_POSY) {
			y = x = 0;
			oled_clear();
		}

        oled_show_char(x, y, *p, 12, 1);
        x += 8;
        p++;
    }

	oled_update_gram();
}

void oled_init(void)
{
	oled_io_init();

	Delay(100000);

	OLED_RST_HIGH();

	oled_write_byte(0xAE, OLED_CMD); //关闭显示

#if 1
	oled_write_byte(0xD5, OLED_CMD); //设置时钟分频因子,震荡频率
	oled_write_byte(0x80, OLED_CMD); //[3:0],分频因子;[7:4],震荡频率

	oled_write_byte(0xA8, OLED_CMD); //设置驱动路数
	oled_write_byte(0X3F, OLED_CMD); //默认0X3F(1/64)

	oled_write_byte(0xD3, OLED_CMD); //设置显示偏移
	oled_write_byte(0X00, OLED_CMD); //默认为0

	oled_write_byte(0x40, OLED_CMD); //设置显示开始行 [5:0],行数.

	oled_write_byte(0x8D, OLED_CMD); //电荷泵设置
	oled_write_byte(0x14, OLED_CMD); //bit2，开启/关闭

	oled_write_byte(0x20, OLED_CMD); //设置内存地址模式
	oled_write_byte(0x02, OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;

	oled_write_byte(0xA1, OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
	oled_write_byte(0xC0, OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数

	oled_write_byte(0xDA, OLED_CMD); //设置COM硬件引脚配置
	oled_write_byte(0x12, OLED_CMD); //[5:4]配置

	oled_write_byte(0x81, OLED_CMD); //对比度设置
	oled_write_byte(0xCF, OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)

	oled_write_byte(0xD9, OLED_CMD); //设置预充电周期
	oled_write_byte(0xF1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;

	oled_write_byte(0xDB, OLED_CMD); //设置VCOMH 电压倍率
	oled_write_byte(0x40, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	oled_write_byte(0xA4, OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)

	oled_write_byte(0xA6, OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示	    

	oled_write_byte(0xAF, OLED_CMD); //开启显示

	oled_clear();

//	oled_fill(0, 0, 127, 63, 1);
#endif
}
