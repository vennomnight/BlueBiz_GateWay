/*
 * Count_Sensor.cpp
 *
 * Created: 2017-08-23 오후 1:45:14
 *  Author: bluebiz
 */ 
#include "Count_Sensor.h"


#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)
void Count_Sensor::Device_Init()
{
	//cbi(DDRD,0); //PORTD 0PIN INPUT
	//sbi(PORTD,0); //USE PULL UP
	DDRD = 0x00;
	PORTD = 0xFF;
	sbi(EICRA,1);
	cbi(EICRA,0);  // ISC10 = 1 ISC00 = 0;
	sbi(EIMSK,0);  // 0Pin interrupt Enable;
}
void* Count_Sensor::operator new(size_t size)
{
	return malloc(size);
}
void Count_Sensor::operator delete(void* ptr)
{
	free(ptr);
}