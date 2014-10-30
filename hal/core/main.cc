#include <stdio.h>
#include "bus.h"
#include "init.h"

int main(int argc, char **argv)
{
	char buf[16];

	global_init();
	bus_xfer(BUS(SPI, 0), buf, 16, OUT);
	bus_xfer(BUS(SPI, 1), buf, 16, OUT);

	return 0;
}
