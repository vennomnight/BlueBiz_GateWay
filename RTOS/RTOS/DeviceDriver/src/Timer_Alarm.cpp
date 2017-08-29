/*
 * Timer_Alarm.cpp
 *
 * Created: 2017-08-24 오후 7:16:00
 *  Author: bluebiz
 */ 
#include "Timer_Alarm.h"
#include <avr/io.h>
#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)
Timer_Alarm::Timer_Alarm()
{
	TCCR2 = 0b00001100; //CTC MODE   1/256분주
	OCR2 = 249; //4ms마다 인터럽트
	sbi(TIMSK,OCIE2); // 인터럽트 활성화	
	//Timer_Alarm::ptr = mem4;
}
void Timer_Alarm::Service_routine()
{
	mem4[3]++;
}
