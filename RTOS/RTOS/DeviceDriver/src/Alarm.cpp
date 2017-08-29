/*
 * Alarm.cpp
 *
 * Created: 2017-08-23 오후 3:34:22
 *  Author: bluebiz
 */ 
#include "Alarm.h"

#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)


typedef struct
{
	uint16_t Period;
	uint16_t Time;
	Alarm_Handle_t alarm_Handle;
}Alarm_State_t;

Alarm_State_t AlarmTable[ALARM_MAX] =
{
	{.Period = 0,.Time = 0, .alarm_Handle = NULL},
};

void Alarm_Init(void)
{
	TCCR0 = 0b00001100; //CTC MODE   1/256분주 
	OCR0 = 124; //2ms마다 인터럽트
	sbi(TIMSK,OCIE0); // 인터럽트 활성화
}
void Alarm_Stop(void)
{
	cbi(TIMSK,OCIE0); //인터럽트 중지
}
void Alarm_Start(void)
{
	sbi(TIMSK,OCIE0); // 인터럽트 활성화
}
void Alarm_Open(Alarm_t Alarm,uint16_t msPeriod,Alarm_Handle_t Handle)
{
	AlarmTable[Alarm].Period = msPeriod;
	//AlarmTable[Alarm].Period = msPeriod / 2;
	AlarmTable[Alarm].Time = 0;
	AlarmTable[Alarm].alarm_Handle = Handle;
}
void Alarm_Close(Alarm_t Alarm)
{
	AlarmTable[Alarm].alarm_Handle = NULL;
}
/*5ms마다 호출됨 */
void TIMER0_COMP_vect(void)
{
	uint8_t i;
	for(i=0;i<ALARM_MAX;i++)
	{
		if(AlarmTable[i].alarm_Handle)
		{
			AlarmTable[i].Time++;
			if(AlarmTable[i].Period == AlarmTable[i].Time)
			{
				AlarmTable[i].Time = 0;
				AlarmTable[i].alarm_Handle();
				AlarmTable[i].alarm_Handle = NULL;
			}
		}
	}
}

