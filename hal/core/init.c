#include "init.h"

extern initcall_t __init_start, __init_end;

void model_init()
{
	initcall_t *fn;

	serial_println("xxxxxxxxxxxxxxx %p %p", &__init_start, &__init_end);

	for (fn = &__init_start; fn < &__init_end; fn++) {
		serial_println("fn: %p", fn);
		(*fn)();
	}

	serial_println("return from model_init");
}


