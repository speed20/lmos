#include "bt.h"
#include "log.h"
#include "stm32f4xx_flash.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define BT_CONFIG_ADDR 0x080FFF00

typedef struct {
	uint32_t baudrate;
	unsigned char pin[5];
	unsigned char name[16];
}Bt_Config_Structure;

static uint8_t bt_port = -1;

int Bt_Config(uint8_t port, uint32_t baudrate, unsigned char *pin, unsigned char *name)
{
	printk("\r\nbegin bt config");

	unsigned char buf[32], resp[32], index;
	uint32_t len, i, n;
	uint32_t *tmp;
	Bt_Config_Structure bt_conf;
	uint8_t update = 0;
	
	tmp = (uint32_t *)&bt_conf;
	for (i=0; i<sizeof(bt_conf)/4; i++) {
		tmp[i] = ((volatile uint32_t *)BT_CONFIG_ADDR)[i];
	}

	printk("stored info: %d %s %s", bt_conf.baudrate, bt_conf.name, bt_conf.pin);

	/* Initialize serial port usart2 */
	serial_init(port, baudrate);

	Delay(1000000);
	/* send test command */
	serial_write(port, "AT", 2);
	serial_read(port, buf, 2);

	if (strncmp(buf, "OK", 2) != 0) {
		printk("failed with response: %s", buf);
		return -1;
	}

	if (bt_conf.baudrate != baudrate) {
		switch (baudrate) {
			case 1200:
				index = 1;
				break;
			case 2400:
				index = 2;
				break;
			case 4800:
				index = 3;
				break;
			case 9600:
				index = 4;
				break;
			case 19200:
				index = 5;
				break;
			case 38400:
				index = 6;
				break;
			case 57600:
				index = 7;
				break;
			case 115200:
				index = 8;
				break;
			default:
				index = 8;
				break;
		}

		/* set speed */
		len = sprintf(buf, "AT+BAUD%d", index);
		n = sprintf(resp, "OK%d", baudrate);

		Delay(1000000);
		serial_write(port, buf, len);
		serial_read(port, buf, n);

		if (strncmp(buf, resp, n) != 0) {
			printk("failed with response: %s", buf);
			return -1;
		}

		update = 1;
		printk("baudrate %d", bt_conf.baudrate);
	}

	if (strcmp(bt_conf.name, name) != 0) {
		/* set name */
		len = sprintf(buf, "AT+NAME%s", name);

		Delay(1000000);
		serial_write(port, buf, len);
		serial_read(port, buf, 9);

		if (strncmp(buf, "OKsetname", 9) != 0) {
			printk("failed with response: %s", buf);
			return -1;
		}
		update = 1;
	}

	if (strcmp(bt_conf.pin, pin) != 0) {
		/* set pin code */
		len = sprintf(buf, "AT+PIN%s", pin);

		Delay(1000000);
		serial_write(port, buf, len);
		serial_read(port, buf, 8);

		if (strncmp(buf, "OKsetPIN", 8) != 0) {
			printk("failed with response: %s", buf);
			return -1;
		}
		update = 1;
	}

	bt_port = port;

	if (update) {
		memset(&bt_conf, 0, sizeof(bt_conf));
		bt_conf.baudrate = baudrate;
		strcpy(bt_conf.pin, pin);
		strcpy(bt_conf.name, name);

		FLASH_Unlock();
		FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
		tmp = (uint32_t *)&bt_conf;
		for (i=0; i<sizeof(bt_conf)/4; i++) {
			FLASH_ProgramWord(BT_CONFIG_ADDR + i*4, tmp[i]);
		}
		FLASH_Lock();
	}

	printk("bt config ok");
	return 0;
}


void Bt_Send(uint8_t *buf, uint32_t len)
{
	serial_write(bt_port, buf, len);
}

void Bt_Recv(uint8_t *buf, uint32_t len)
{
	serial_read(bt_port, buf, len);
}
