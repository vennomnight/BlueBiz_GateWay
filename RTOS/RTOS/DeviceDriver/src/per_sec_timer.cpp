/*
 * per_sec_timer.cpp
 *
 * Created: 2017-07-25 오후 3:18:57
 *  Author: bluebiz
 */ 
#include "per_sec_timer.h"

Timer::Timer()
{
	
}
void Timer::Device_Init()
{
	TCCR3A = 0x40;
	TCCR3B = 0x04;
	OCR3A = 15624;
	ETIMSK = 1 << OCIE3C;
}
void* Timer::operator new(size_t size)
{
	return malloc(size);
}
void Timer::operator delete(void* ptr)
{
	free(ptr);
}