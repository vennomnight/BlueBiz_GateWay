/*
 * Adc.h
 *
 * Created: 2017-10-17 오후 2:51:00
 *  Author: bluebiz
 */ 


#ifndef ADC_H_
#define ADC_H_

#include "DeviceDriverInterface.h"
#include "FreeRTOS.h"
#include "semphr.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

class Adc : public DeviceDriveInterFace
{
	private:
	uint8_t channel_inf;
	uint8_t read_flag;
	public:
	void Device_Init(); //디바이스 초기화
	void Start_Device(uint8_t flag); //채널번호 
	char Device_Read() const; //채널 번호 리턴 
	void Read_Set(); //채널을 읽어 가면 셋을 해야함.
	void* operator new(size_t size);
	void operator delete(void* ptr);
};


#endif /* ADC_H_ */