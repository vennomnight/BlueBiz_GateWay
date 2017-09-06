/*
 * UartDriver.h
 *
 * Created: 2016-12-02 오후 9:18:41
 *  Author: kimkisu
 */ 


#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "DeviceDriverInterface.h"
#include "Ubbr_Calculate.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

class DeviceDriveInterFace;

class UartDriver : public DeviceDriveInterFace, private Ubbr
{
	using Ubbr::Ubbr_Value;
	private:
	static const uint8_t UX1 = 0;
	uint16_t Uart_baudrate = 4800;
	SemaphoreHandle_t char_Mutex;
	SemaphoreHandle_t Uart_Mutex;
	static UartDriver* inst;
	uint16_t Ubbr_Value(const uint16_t &Uart_baudrate);
	void UART_Putchar(const char data);
	void UART_PutString(const char *str);
	public:
	explicit UartDriver();
	explicit UartDriver(uint16_t Uart_baudrate);
	void Device_Init();
	void* operator new(size_t size);
	void operator delete(void* ptr);
	void Device_Writes(const char* data);
	void Device_Write(char data);
	void Stop_Device(void) const;
	void Start_Device(void) const;
	static const UartDriver* const getInstance();
	
};


#endif /* UARTDRIVER_H_ */