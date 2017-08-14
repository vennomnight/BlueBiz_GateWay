/*
 * RS485Driver.h
 *
 * Created: 2017-01-01 오후 4:07:04
 *  Author: kimkisu
 */ 


#ifndef RS485DRIVER_H_
#define RS485DRIVER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "DeviceDriverInterface.h"
#include "semphr.h"
#include "queue.h"

#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)
#define F_CPU 16000000UL
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define USART_BAUDRATE 9600


class DeviceDriveInterFace;

class RS485Driver : public DeviceDriveInterFace
{
	private:
	SemaphoreHandle_t char_Mutex;
	SemaphoreHandle_t Uart_Mutex;
	static RS485Driver* inst;
	void UART_Putchar(const char data);
	void UART_PutString(const char *str);
	public:
	explicit RS485Driver();
	void Device_Init();
	void* operator new(size_t size);
	void operator delete(void* ptr);
	void Device_Writes(const char* data);
	void Device_Write(char data);
	static const RS485Driver* const getInstance();
	
};



#endif /* RS485DRIVER_H_ */