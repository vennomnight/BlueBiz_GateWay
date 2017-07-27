/*
 * Channel.h
 *
 * Created: 2016-12-01 오후 10:00:34
 *  Author: kimkisu
 */ 


#ifndef CHANNEL_H_
#define CHANNEL_H_
#include "Request.h"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "queue.h"
#define MAX_REQUEST 10
#define Worker 1
class WorkerThread;
class Channel
{
private:
	Request requestQueue[MAX_REQUEST];
	TaskHandle_t xHandle[Worker];
	int tail;
	int head;
	int count;
	SemaphoreHandle_t *Channel_Mutex;
    WorkerThread *threadPool[Worker];
	SemaphoreHandle_t Mutex;
	void wait();
	void notifyAll();
public:
	int ProcCount();
	Channel(int threads);
	void startWorkers();
	void putRequest(Request request);
	Request takeRequest();
};


#endif /* CHANNEL_H_ */