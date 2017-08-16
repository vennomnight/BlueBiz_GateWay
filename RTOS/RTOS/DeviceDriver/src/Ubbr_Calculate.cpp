/*
 * Ubbr_Calculate.cpp
 *
 * Created: 2017-08-16 오전 11:28:39
 *  Author: bluebiz
 */ 
#include "Ubbr_Calculate.h"

uint16_t Ubbr::Ubbr_Value(const uint16_t &Uart_baudrate)
{
	float temp =  (((F_cpu / (Uart_baudrate * 16UL))) - 1);
	float ubbr_h = (uint16_t)(temp + 0.5);
	float ubbr_l = (uint8_t)(temp + 0.5);
	uint16_t result = (0xff00 & ((uint16_t)ubbr_h << 8)) | (0x00ff & (uint8_t)ubbr_l);
	return result;
}
