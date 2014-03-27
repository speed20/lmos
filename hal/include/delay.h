#include "stm32f4xx.h"

extern volatile uint8_t delay_flag;

void Delay_Config(void);
void Delay(uint32_t usec);
