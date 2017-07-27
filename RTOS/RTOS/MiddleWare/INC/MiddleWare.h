/*
 * MiddleWare.h
 *
 * Created: 2016-12-04 오후 3:08:22
 *  Author: kimkisu
 */ 


#ifndef MIDDLEWARE_H_
#define MIDDLEWARE_H_
#include <avr/io.h>

typedef enum
{
	EVENT_UART,
	EVENT_CAP_INPUT,
	ADC_0,
	ADC_1,
	ADC_2,
	ADC_3
}Event_t;

typedef void(*Event_Callback_t)(Event_t Event,uint16_t Arg);
void Event_Init(void);
void Event_RegisterCallback(Event_t Event,uint16_t Arg,Event_Callback_t Callback);
void Event_RemoveCallback(Event_t Event);
static void Put_Event(Event_t Event,uint16_t Arg);
void Event_Process(void);

static void Adc_00(uint16_t Data);
static void Adc_01(uint16_t Data);
static void Adc_02(uint16_t Data);
static void Adc_03(uint16_t Data);





#endif /* MIDDLEWARE_H_ */