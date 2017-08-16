/*
 * RS485Driver.cpp
 *
 * Created: 2017-01-01 오후 4:07:52
 *  Author: kimkisu
 */ 
#include "RS485Driver.h"

RS485Driver* RS485Driver::inst = nullptr;

RS485Driver::RS485Driver()
{
	if(inst == nullptr)
		inst = this;
}
uint16_t RS485Driver::Ubbr_Value(const uint16_t &_Uart_baudrate) 
{
	return Ubbr::Ubbr_Value(_Uart_baudrate);
}

void RS485Driver::Device_Init()
{
	/*Data : 8bit,Parity : None ,Stopbit:1bit Baudrate:9600bps*/
	UCSR1A=0x00;
    UCSR1B = 0x98;
	UCSR1C = 0x06;
	uint16_t reg = Ubbr_Value(this->Uart_baudrate);
	UBRR1H = reg << 8;
	UBRR1L = reg >> 8;
	Uart_Mutex = xSemaphoreCreateMutex();
	char_Mutex= xSemaphoreCreateMutex();
}
RS485Driver::RS485Driver(uint16_t _Uart_baudrate)
{
	this->Uart_baudrate = _Uart_baudrate;
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

const RS485Driver* const RS485Driver::getInstance()
{
	if (inst == nullptr)
		inst = new RS485Driver();
	
	return inst;
}