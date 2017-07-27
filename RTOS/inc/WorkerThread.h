/*
 * WorkerThread.h
 *
 * Created: 2016-12-01 오후 10:06:44
 *  Author: kimkisu
 */ 


#ifndef WORKERTHREAD_H_
#define WORKERTHREAD_H_
#include "Channel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "queue.h"
	
class Request;
class Channel;
class WorkerThread;
typedef void (*thread)(WorkerThread* ptr,Channel *channel,TaskHandle_t *xHandles);
class WorkerThread
{
private:
	Channel *channel;
	TaskHandle_t xHandles;

	thread th = NULL;
	//void ThreadCreate(Channel *channel);
public:
	WorkerThread();
	WorkerThread(Channel *channel);
	void* operator new(size_t size);
	void start(TaskHandle_t *xHandle);
	static void run(void* pvParam);
};


#endif /* WORKERTHREAD_H_ */