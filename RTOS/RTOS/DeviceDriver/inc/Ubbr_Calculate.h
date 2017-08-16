/*
 * Ubbr_Calculate.h
 *
 * Created: 2017-08-16 오전 11:25:19
 *  Author: bluebiz
 */ 


#ifndef UBBR_CALCULATE_H_
#define UBBR_CALCULATE_H_
#include "FreeRTOS.h"
class Ubbr
{	
protected:
	const static uint32_t F_cpu = 16000000UL;
	uint16_t Ubbr_Value(const uint16_t &Uart_baudrate);		
};

#endif /* UBBR_CALCULATE_H_ */