#include "stm32f4xx.h"

int Bt_Config(uint8_t port, uint32_t baudrate, unsigned char *pin, unsigned char *name);
void Bt_Send(uint8_t *buf, uint32_t len);
void Bt_Recv(uint8_t *buf, uint32_t len);
