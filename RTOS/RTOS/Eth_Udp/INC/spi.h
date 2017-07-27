

#ifndef _SPI_H
#define _SPI_H

//#include <stdint.h>
#include "config.h"
#include "avr_compat.h"
//#define noinline __attribute__((noinline))

/* prototypes */
extern void spi_init(void);
//void noinline spi_wait_busy(void);
//uint8_t noinline spi_send(uint8_t data);


#endif
