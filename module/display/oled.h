#include "stm32f4xx.h"

#define OLED_SPI                       SPI1
#define OLED_SPI_CLK                   RCC_APB2Periph_SPI1

#define OLED_SPI_SCK_PIN               GPIO_Pin_5                  /* PA.05 */
#define OLED_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
#define OLED_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define OLED_SPI_SCK_SOURCE            GPIO_PinSource5
#define OLED_SPI_SCK_AF                GPIO_AF_SPI1

#define OLED_SPI_MISO_PIN              GPIO_Pin_6                  /* PA.6 */
#define OLED_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
#define OLED_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define OLED_SPI_MISO_SOURCE           GPIO_PinSource6
#define OLED_SPI_MISO_AF               GPIO_AF_SPI1

#define OLED_SPI_MOSI_PIN              GPIO_Pin_7                  /* PA.7 */
#define OLED_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
#define OLED_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define OLED_SPI_MOSI_SOURCE           GPIO_PinSource7
#define OLED_SPI_MOSI_AF               GPIO_AF_SPI1

#define OLED_SPI_CS_PIN                GPIO_Pin_11                  /* PE.11 */
#define OLED_SPI_CS_GPIO_PORT          GPIOE                       /* GPIOE */
#define OLED_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOE

#define OLED_DC_PIN					   GPIO_Pin_12                  /* PE.12 */
#define OLED_DC_GPIO_PORT			   GPIOE                       /* GPIOE */
#define OLED_DC_GPIO_CLK			   RCC_AHB1Periph_GPIOE

#define OLED_RST_PIN				   GPIO_Pin_13                  /* PE.13 */
#define OLED_RST_GPIO_PORT			   GPIOE                       /* GPIOE */
#define OLED_RST_GPIO_CLK			   RCC_AHB1Periph_GPIOE

#define OLED_CS_LOW()       GPIO_ResetBits(OLED_SPI_CS_GPIO_PORT, OLED_SPI_CS_PIN)
#define OLED_CS_HIGH()      GPIO_SetBits(OLED_SPI_CS_GPIO_PORT, OLED_SPI_CS_PIN)

#define OLED_DC_LOW()       GPIO_ResetBits(OLED_DC_GPIO_PORT, OLED_DC_PIN)
#define OLED_DC_HIGH()      GPIO_SetBits(OLED_DC_GPIO_PORT, OLED_DC_PIN)

#define OLED_RST_LOW()       GPIO_ResetBits(OLED_RST_GPIO_PORT, OLED_RST_PIN)
#define OLED_RST_HIGH()      GPIO_SetBits(OLED_RST_GPIO_PORT, OLED_RST_PIN)

void oled_init(void);
void oled_show_char(uint8_t x, uint8_t y, uint8_t ch, uint8_t size, uint8_t mode);
void oled_show_string(uint8_t x,uint8_t y,const uint8_t *p);
void oled_clear();
