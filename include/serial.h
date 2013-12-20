#include "stm32f4_discovery.h"

void serial_init(uint32_t port, uint32_t baudrate);
void usart_print(uint32_t port, char *str);
int16_t serial_print(const char * format, ...);
int16_t serial_println(const char * format, ...);
void serial_read(uint32_t port, uint8_t *buf, uint32_t len);
void serial_write(uint32_t port, uint8_t *buf, uint32_t len);

