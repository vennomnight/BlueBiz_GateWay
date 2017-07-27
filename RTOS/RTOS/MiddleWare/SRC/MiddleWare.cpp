/*
 * MiddleWare.cpp
 *
 * Created: 2016-12-04 오후 3:08:34
 *  Author: kimkisu
 */ 
#include "MiddleWare.h"
#define EVENT_BUFFER_SIZE 128
typedef struct
{
	Event_t buf[EVENT_BUFFER_SIZE];
	uint16_t arg[EVENT_BUFFER_SIZE];
	uint8_t top;
	uint8_t bottom;
	uint8_t count;

}Event_Buffer_t;

static void Event_Uart(uint8_t Data);
static void Event_Timer(uint16_t Data);

static uint8_t Event_GetEvent(Event_t *Event,uint16_t *Arg);


Event_Buffer_t EventBuf;
Event_Callback_t EventCallbackTable[EVENT_BUFFER_SIZE] = {0};


void Event_Init(void)
{
	EventBuf.bottom = 0;
	EventBuf.top = 0;
	EventBuf.count = 0;

}
void Event_RegisterCallback(Event_t Event,uint16_t Arg,Event_Callback_t Callback)
{
	switch(Event)
	{
		case EVENT_UART:
		
		break;
		case EVENT_CAP_INPUT :

		break;
		case ADC_0:

		break;
		case ADC_1:

		break;
		case ADC_2:

		break;
		case ADC_3:

		break;
		default:
		return;
	}
	EventCallbackTable[Event] = Callback;
}
void Event_RemoveCallback(Event_t Event)
{
	EventCallbackTable[Event] = 0;
}
static void Put_Event(Event_t Event,uint16_t Arg)
{
	EventBuf.buf[EventBuf.top] = Event;
	EventBuf.arg[EventBuf.top] = Arg;
	EventBuf.top = (EventBuf.top + 1) % EVENT_BUFFER_SIZE;
	EventBuf.count++;
	if(EventBuf.count > EVENT_BUFFER_SIZE)
	{
		EventBuf.bottom = EventBuf.top;
		EventBuf.count = EVENT_BUFFER_SIZE;
	}
	
}
static uint8_t Event_GetEvent(Event_t *Event,uint16_t *Arg)
{
	if(EventBuf.count == 0)
	{
		return 0;
	}
	else
	{
		*Event = EventBuf.buf[EventBuf.bottom];
		*Arg = EventBuf.arg[EventBuf.bottom];
		EventBuf.bottom = (EventBuf.bottom + 1) % EVENT_BUFFER_SIZE;
		EventBuf.count--;
		return 1;
	}
}


void Event_Process(void)
{
	Event_t Event;
	uint16_t Arg;
	if(Event_GetEvent(&Event,&Arg) == 0)
	{
		return;
	}
	else
	{
		EventCallbackTable[Event](Event,Arg);
	}
}
static void Event_Uart(uint8_t Data)
{
	//Put_Event(EVENT_UART,Data); //문제생길시 변경 ...
}
static void Event_Timer(uint16_t Data)
{
	Put_Event(EVENT_CAP_INPUT,Data);
}

static void Adc_00(uint16_t Data)
{
	Put_Event(ADC_0,Data);
}
static void Adc_01(uint16_t Data)
{
	Put_Event(ADC_1,Data);
}
static void Adc_02(uint16_t Data)
{
	Put_Event(ADC_2,Data);
}
static void Adc_03(uint16_t Data)
{
	Put_Event(ADC_3,Data);
}