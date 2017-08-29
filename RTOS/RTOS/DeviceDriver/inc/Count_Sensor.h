/*
 * Count_Sensor.h
 *
 * Created: 2017-08-23 오후 1:44:11
 *  Author: bluebiz
 */ 


#ifndef COUNT_SENSOR_H_
#define COUNT_SENSOR_H_
#include "DeviceDriverInterface.h"
#include "FreeRTOS.h"
#include "semphr.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

class Count_Sensor : public DeviceDriveInterFace
{
	public:
		void Device_Init();
		void* operator new(size_t size);
		void operator delete(void* ptr);
};



#endif /* COUNT_SENSOR_H_ */