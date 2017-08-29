/*
 * Alarm.h
 *
 * Created: 2017-08-23 오후 3:29:41
 *  Author: bluebiz
 */ 


#ifndef ALARM_H_
#define ALARM_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "semphr.h"



extern "C"
{
typedef void(*Alarm_Handle_t)(void);
void TIMER0_COMP_vect(void) __attribute__ ((signal));

typedef enum
{
	ALARM0,
	ALARM1,
	ALARM2,
	ALARM3,
	ALARM4,
	ALARM_MAX
}Alarm_t;

typedef void(*Alarm_Handle_t)(void);

void Alarm_Init(void);
void Alarm_Open(Alarm_t Alarm,uint16_t msPeriod,Alarm_Handle_t Handle);
void Alarm_Close(Alarm_t Alarm);
void Alarm_Stop(void);
void Alarm_Start(void);


}
#endif /* ALARM_H_ */