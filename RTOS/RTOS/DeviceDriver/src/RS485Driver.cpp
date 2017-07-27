/*
 * RS485Driver.cpp
 *
 * Created: 2017-01-01 오후 4:07:52
 *  Author: kimkisu
 */ 
#include "RS485Driver.h"
#include "avr/interrupt.h"
#include "Dev_Manager.h"
RS485Driver* RS485Driver::inst = nullptr;

RS485Driver::RS485Driver()
{
	if(inst == nullptr)
		inst = this;
}
void RS485Driver::Device_Init()
{
	/*Data : 8bit,Parity : None ,Stopbit:1bit Baudrate:9600bps*/
	UCSR1A=0x00;
    UCSR1B = 0x98;
	UCSR1C = 0x06;
	UBRR1H = (uint8_t)(UBRR_VALUE>>8);
	UBRR1L = (uint8_t) UBRR_VALUE;
    //UBRR1H=0x00;
    //UBRR1L=0x67;
	Uart_Mutex = xSemaphoreCreateMutex();
	char_Mutex= xSemaphoreCreateMutex();
	//DDRF=0X80; //TX or RX Enable Pin
	//PORTF=0X00;	//RS485 rx Enable
}

void RS485Driver::operator delete(void* ptr)
{
	free(ptr);
}
void* RS485Driver::operator new(size_t size)
{
	return malloc(size);
}
void RS485Driver::UART_Putchar(const char data)
{
	if(xSemaphoreTake(char_Mutex,100) == pdPASS)
	{
		while((UCSR1A & (1 << UDRE1)) == 0);
		UDR1 = data;
		xSemaphoreGive(char_Mutex);
	}
}
void RS485Driver::UART_PutString(const char *str)
{
	if(xSemaphoreTake(Uart_Mutex,100) == pdPASS)
	{
		while(*str)
		{
			UART_Putchar(*(str)++);
		}
		xSemaphoreGive(Uart_Mutex);
	}
}
void RS485Driver::Device_Write(char data)
{
	UART_Putchar(data);
}
void RS485Driver::Device_Writes(const char* data)
{
	UART_PutString(data);
}

RS485Driver* RS485Driver::getInstance()
{
	if (inst == nullptr)
		inst = new RS485Driver();
	
	return inst;
}