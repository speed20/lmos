#include "global_includes.h"

#define PRINT_TO_USART
#define PRINT_PORT 0
#define BAUDRATE 115200

#define SERIAL_USE_DMA
void serial_init(uint8_t port, uint32_t baudrate);
void usart_print(uint8_t port, char *str);
int16_t serial_print(const char * format, ...);
int16_t serial_println(const char * format, ...);
void serial_read(uint8_t port, uint8_t *buf, uint32_t len);
void serial_write(uint8_t port, uint8_t *buf, uint32_t len);

