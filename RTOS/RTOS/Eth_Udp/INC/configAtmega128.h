#ifndef _CONFIG_H
#define _CONFIG_H

//atmega128
//#include <stdint.h>
#define F_CPU 16000000UL  // 16 MHz
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB

#define LED_PORT    PORTB
#define LED_DDR     DDRB

#define SWITCH1_PORT PORTB
#define SWITCH1_DDR  DDRB

#define INT0_INPUT_DDR DDRC
#define INT0_INPUT_DDC2 DDC2

#define SPI_CS      PB0
#define SPI_MOSI    PB2
#define SPI_SCK     PB1
#define SPI_MISO    PB3

#define STATUS_LED  PB6
#define STATUS_LED_DD DDB6
#define SWITCH1     PB7     
#define SWITCH1_DD DDB7


#endif
