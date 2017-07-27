/*
 * per_sec_timer.h
 *
 * Created: 2017-07-25 오후 3:14:25
 *  Author: bluebiz
 */ 


#ifndef PER_SEC_TIMER_H_
#define PER_SEC_TIMER_H_
#include "FreeRTOS.h"
#include "task.h"
#include "DeviceDriverInterface.h"
#include "semphr.h"
#include "queue.h"
#include "avr/interrupt.h"
#include "Dev_Manager.h"

class Timer : public DeviceDriveInterFace
{
public:
	Timer();
	void Device_Init();
	void* operator new(size_t size);
	void operator delete(void* ptr);
};



#endif /* PER_SEC_TIMER_H_ */