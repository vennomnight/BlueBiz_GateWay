/*
 * Dev_Manager.h
 *
 * Created: 2016-12-02 오후 9:20:37
 *  Author: kimkisu
 */ 


#ifndef DEV_MANAGER_H_
#define DEV_MANAGER_H_
#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "DeviceDriverInterface.h"
#include "semphr.h"
#include "queue.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)
#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)


typedef enum
{
	UART0,
	RS485,
	SEC_TIMER,
	MAX
}Dev_type;

typedef void(*ISR_Handle)(Dev_type Device,uint16_t Arg);

class DeviceDriveInterFace;
class Dev_Manager
{
	private:
	static Dev_Manager* inst;
	
	DeviceDriveInterFace *interface[MAX] = {nullptr};
	public:
	ISR_Handle isr_handle[MAX] = {nullptr};
	DeviceDriveInterFace* getInterfaceAddr(Dev_type Device);
	Dev_Manager();
	~Dev_Manager();
	void Open_Handle(Dev_type Device,ISR_Handle _isr_handle);
	void Close_Handle(Dev_type Device);
	void* operator new(size_t size);
	void operator delete(void* ptr);
	void Register_Dev(DeviceDriveInterFace *_interface,Dev_type Dev);
	void Device_Init(Dev_type Device);
	void Write(Dev_type Device,char data);
	void Writes(Dev_type Device,char* data);
	char Driver_Check(Dev_type Device);

	static Dev_Manager* getInstance();

	
};


#endif /* DEV_MANAGER_H_ */