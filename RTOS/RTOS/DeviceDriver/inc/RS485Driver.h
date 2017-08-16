/*
 * RS485Driver.h
 *
 * Created: 2017-01-01 오후 4:07:04
 *  Author: kimkisu
 */ 


#ifndef RS485DRIVER_H_
#define RS485DRIVER_H_
#include "FreeRTOS.h"
#include "semphr.h"
#include "DeviceDriverInterface.h"
#include "Ubbr_Calculate.h"

#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

class RS485Driver : public DeviceDriveInterFace , private Ubbr
{
	using Ubbr::Ubbr_Value;
	private:
	uint16_t Uart_baudrate = 9600;
	uint16_t Ubbr_Value(const uint16_t &Uart_baudrate);
	SemaphoreHandle_t char_Mutex;
	SemaphoreHandle_t Uart_Mutex;
	static RS485Driver* inst;
	void UART_Putchar(const char data);
	void UART_PutString(const char *str);
	public:
	explicit RS485Driver();
	explicit RS485Driver(uint16_t Uart_baudrate);
	void Device_Init();
	void* operator new(size_t size);
	void operator delete(void* ptr);
	void Device_Writes(const char* data);
	void Device_Write(char data);
	static const RS485Driver* const getInstance();
	
};



#endif /* RS485DRIVER_H_ */