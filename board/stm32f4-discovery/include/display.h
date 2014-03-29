#include "stm32f429i_discovery_lcd.h"

#define MESSAGE1   "ADC conversion w/DMA"
#define MESSAGE1_1 "continuouslyTransfer" 
#define MESSAGE2   " ADC Ch13 Conv   "
#define MESSAGE2_1 "    2.4Msps      "
#define MESSAGE5   " ADC3 = %d,%1d V "
#define LINENUM            0x15
#define FONTSIZE         Font12x12

void Display_Init(void);
