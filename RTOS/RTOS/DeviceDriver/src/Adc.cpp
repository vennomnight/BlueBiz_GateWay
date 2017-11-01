/*
 * Adc.cpp
 *
 * Created: 2017-10-17 오후 2:59:35
 *  Author: bluebiz
 */ 

#include "Adc.h"

void Adc::Device_Init() //AVCC
{
	ADMUX = 0b01000000; //우측 정렬 //AVCC 사용 
	//ADCSRA = 0x8F; //ADC활성화 단일변환모드 인터럽트 허용 1/8 분주비
	//ADCSRA = 0xA7; 
	ADCSRA = 0b10000111; //ADC ENABLE
}
void Adc::Start_Device(uint8_t flag) //flag == 채널 
{
	while(ADCSRA & 0x40);
	ADMUX = (ADMUX & 0xe0) | flag;
	ADCSRA |= (1 << ADSC);	
}
char Adc::Device_Read() const  //채널값 리턴,
{
	return channel_inf;
}
void Adc::Read_Set()
{
	read_flag = 0;
}
void* Adc::operator new(size_t size)
{
	return malloc(size);
}
void Adc::operator delete(void* ptr)
{
	free(ptr);
}