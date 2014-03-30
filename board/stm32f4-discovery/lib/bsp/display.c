#include "display.h"

void Display_Init(void)
{
  /* Initialize the LCD */
  LCD_Init();
//  LCD_LayerInit();
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
}
