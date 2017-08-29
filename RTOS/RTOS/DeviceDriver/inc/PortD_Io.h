/*
 * PortD_Io.h
 *
 * Created: 2017-08-23 오전 10:42:10
 *  Author: bluebiz
 */ 


#ifndef PORTD_IO_H_
#define PORTD_IO_H_

#include "DeviceDriverInterface.h"

#include "FreeRTOS.h"
#include "semphr.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)
	 typedef enum
	 {
		 NULL_SM,
		 PULL_UP,
		 HIGH_IMPEDANCE
	 }Set_MODE;
	 typedef enum
	 {
		 NULL_GM,
		 INPUT,
		 OUTPUT
	 }GPIO_MODE;
	 typedef enum
	 {
		 NULL_IM,
		 NONE_INTERRUPT,
		 USE_INTERRUPT
	 }Interrupt_Mode;
	 typedef enum
	 {
		 NULL_ET,
		 FALLING_EDGE,
		 RISING_EDGE
	 }Edge_Triger;
	 
class Portd_Io : public DeviceDriveInterFace
{
public:

	void Device_Init();
	void Set_Parma(GPIO_MODE gpio,Set_MODE mode,Interrupt_Mode int_mode,Edge_Triger edge_triger);
	void Set_Mode();
	void* operator new(size_t size);
	void operator delete(void* ptr);
 private:
	Set_MODE sm;
	GPIO_MODE gm;
	Interrupt_Mode im;
	Edge_Triger et;
};




#endif /* PORTD_IO_H_ */