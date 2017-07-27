/*
 * UartDriver.cpp
 *
 * Created: 2016-12-02 오후 9:19:54
 *  Author: kimkisu
 */ 
#include "UartDriver.h"
#include "avr/interrupt.h"
#include "Dev_Manager.h"
UartDriver* UartDriver::inst = nullptr;
UartDriver::UartDriver()
{
	if (inst == nullptr)
		inst = this;		
}
UartDriver* UartDriver::getInstance()
{
	if (inst == nullptr)
	  inst = new UartDriver();
	
	return inst;
}

void UartDriver::Device_Init()
{
	/*Data : 8bit,Parity : None ,Stopbit:1bit Baudrate:9600bps*/
	UCSR0B = 0x98;
	UCSR0C = 0x06;
	//UBRR0H=0x00;
	//UBRR0L=0x67;
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t) UBRR_VALUE;

	Uart_Mutex = xSemaphoreCreateMutex();
	char_Mutex= xSemaphoreCreateMutex();
}

void UartDriver::operator delete(void* ptr)
{
	free(ptr);
}
void* UartDriver::operator new(size_t size)
{
	return malloc(size);
}
void UartDriver::UART_Putchar(const char data)
{
	if(xSemaphoreTake(char_Mutex,100) == pdPASS)
	{
		while((UCSR0A & (1 << UDRE0)) == 0);
		UDR0 = data;
		xSemaphoreGive(char_Mutex);
	}
}
void UartDriver::UART_PutString(const char *str)
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
void UartDriver::Device_Write(char data)
{
	UART_Putchar(data);
}
void UartDriver::Device_Writes(const char* data)
{
	UART_PutString(data);
}
