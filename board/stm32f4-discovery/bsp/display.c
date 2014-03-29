#include "display.h"

void Display_Init(void)
{
	int i;
	double tmp;
  /* Initialize the LCD */
  LCD_Init();
  LCD_LayerInit();
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD Background Layer  */
  LCD_SetLayer(LCD_BACKGROUND_LAYER);
  
  /* Clear the Background Layer */ 
  LCD_Clear(LCD_COLOR_WHITE);
  
  /* Configure the transparency for background */
  LCD_SetTransparency(0);
  
  /* Set LCD Foreground Layer  */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);

#if 0
  /* Configure the transparency for foreground */
  LCD_SetTransparency(200);
  
  /* Clear the Foreground Layer */ 
  LCD_Clear(LCD_COLOR_WHITE);
  
  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_BLUE);
  LCD_SetTextColor(LCD_COLOR_WHITE);
  
    /* Set the LCD Text size */
  LCD_SetFont(&FONTSIZE);
  
  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_BLUE);
  LCD_SetTextColor(LCD_COLOR_WHITE);
  
//  LCD_DisplayStringLine(LINE(LINENUM), (uint8_t*)MESSAGE1);
//  LCD_DisplayStringLine(LINE(LINENUM + 1), (uint8_t*)MESSAGE1_1);
//  LCD_DisplayStringLine(LINE(0x17), (uint8_t*)"                               ");
  
  /* Set the LCD Text size */
  LCD_SetFont(&Font16x24);
  
  LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)MESSAGE2);
  LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)MESSAGE2_1);
  
  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_WHITE);
  LCD_SetTextColor(LCD_COLOR_BLUE); 

  LCD_DrawCircle(120, 160, 20);
#endif
#if 0
#define M_2_PI (3.1415926f * 2.0f)
#define M_PI_16 (3.1415926f / 4.0f)

  for (i=0; i<240; i++) {
	  float x;
	  pp[i].X = i;
	  x = (float)i * M_PI_16;
	  while (x > M_2_PI) x -= M_2_PI;

	  pp[i].Y = (uint16_t)(arm_sin_f32(x)*1000  + 100);
	  serial_println("(%d, %d, %d)", pp[i].X, pp[i].Y, (int32_t)arm_sin_f32(x));
  }
	LCD_PolyLine(pp, 240);
#endif
}
