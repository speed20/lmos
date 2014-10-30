#ifndef __INIT_H
#define __INIT_H
typedef int (*initcall_t)(void);

#define __define_initcall(level, fn, id) \
	    initcall_t __initcall_##fn __attribute__((section(".initcall"level".init"))) = fn

#define device_initcall(fn)     __define_initcall("6", fn, 6)

#define driver_init(fn)	device_initcall(fn)

void model_init();
#endif
