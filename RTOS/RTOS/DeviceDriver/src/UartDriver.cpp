/*
 * UartDriver.cpp
 *
 * Created: 2016-12-02 오후 9:19:54
 *  Author: kimkisu
 */ 

#include "UartDriver.h"
#include "avr/interrupt.h"
#include "Dev_Manager.h"
#include "DeviceDriverInterface.h"
#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)

UartDriver* UartDriver::inst = nullptr;
UartDriver::UartDriver()
{
	if (inst == nullptr)
		inst = this;		
}
UartDriver::UartDriver(uint16_t Uart_baudrate)
{
	this->Uart_baudrate = Uart_baudrate;
}
const UartDriver* const UartDriver::getInstance()
{
	if (inst == nullptr)
	  inst = new UartDriver();
	
	return inst;
}

void UartDriver::Device_Init()
{
	/*Data : 8bit,Parity : None ,Stopbit:1bit Baudrate:9600bps*/
	if(UX1)
	{
		UCSR0B = 0x98;
		UCSR0C = 0x06;
		UCSR0A = 0x02;
		//UBRR0H = (uint8_t)(UBRR_VALUE_UX>>8);
		//UBRR0L = (uint8_t) UBRR_VALUE_UX;
	}
	else
	{
		UCSR0B = 0x98;
		UCSR0C = 0x06;
		uint16_t UBRR_VALUE = Ubbr_Value(Uart_baudrate);
		UBRR0H = UBRR_VALUE << 8;
		UBRR0L = UBRR_VALUE >> 8;
	}
	Uart_Mutex = xSemaphoreCreateMutex();
	char_Mutex= xSemaphoreCreateMutex();

}
uint16_t UartDriver::Ubbr_Value(const uint16_t &_Uart_baudrate)
{
	return Ubbr::Ubbr_Value(_Uart_baudrate);
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
void UartDriver::Stop_Device() const
{
	cbi(UCSR0B,RXEN0); //Stop RX
	cbi(UCSR0B,TXEN0); //Stop TX
}
void UartDriver::Start_Device() const
{
	sbi(UCSR0B,RXEN0);
	sbi(UCSR0B,TXEN0);
}