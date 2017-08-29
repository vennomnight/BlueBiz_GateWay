/*
 * PortD_Io.cpp
 *
 * Created: 2017-08-23 오전 10:43:43
 *  Author: bluebiz
 */ 
#include "PortD_Io.h"
#include <avr/io.h>
#include <avr/sfr_defs.h>


#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)

void Portd_Io::Device_Init()
{
	Set_Mode();
}
void Portd_Io::Set_Parma(GPIO_MODE gpio,Set_MODE mode,Interrupt_Mode int_mode,Edge_Triger edge_triger)
{
	this->sm = mode;
	this->gm = gpio;
	this->im = int_mode;
	this->et = edge_triger;
}
void Portd_Io::Set_Mode()
{
	if(this->gm == NULL_GM || this->sm == NULL_SM || this->im == NULL_IM || this->et == NULL_ET)
	{
		return;
	}
	switch(this->gm)
	{
		case INPUT:
			cbi(DDRD,0);
			cbi(DDRD,1);
			break;
		case OUTPUT:
			DDRD = 0xff;
			return;
	}
	switch(this->sm)
	{
		case PULL_UP:
			PORTD = 0xff;
			break;
		case HIGH_IMPEDANCE:
			SFIOR = 0x00;
			PORTD = 0x00; 
			break;
	}
	switch (this->im)
	{
		case NONE_INTERRUPT:
			cbi(EIMSK,0);
			cbi(EIMSK,1);
			break;
		case USE_INTERRUPT:
			sbi(EIMSK,0);
			sbi(EIMSK,1);
			break;
	}
	switch(this->et)
	{
		case FALLING_EDGE:
			sbi(EICRA,1);
			cbi(EICRA,0);
			sbi(EICRA,2);
			cbi(EICRA,3);
			break;
		case RISING_EDGE:
			sbi(EICRA,1);
			sbi(EICRA,0);
			sbi(EICRA,2);
			sbi(EICRA,3);
			break;
	}
}
void* Portd_Io::operator new(size_t size)
{
	return malloc(size);
}
void Portd_Io::operator delete(void* ptr)
{
	free(ptr);
}