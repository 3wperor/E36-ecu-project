#ifndef HAL_IO_H
#define HAL_IO_H

/* Put every output into its safe (off) state. Called first thing in main(),
 * before any timer or interrupt is enabled. */
void io_init_safe(void);

#endif
