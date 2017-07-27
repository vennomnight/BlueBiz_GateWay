/*
 * Channel.cpp
 *
 * Created: 2016-12-01 오후 10:01:20
 *  Author: kimkisu
 */ 
#include "Channel.h" 
#include "WorkerThread.h"
#include "UartDriver.h"
#include "semphr.h"
#include "Dev_Manager.h"
#define MAX_REQUEST 10

void Channel::wait()
{
	for(char i=0;i<Worker;i++)
	{
		vTaskSuspend (xHandle[i]); 
	}
}
int Channel::ProcCount()
{
	return count;
}
void Channel::notifyAll()
{
		for(char i=0;i<Worker;i++)
		{
			vTaskResume(xHandle[i]);
		}
}
Channel::Channel(int threads)
{
	this->head = 0;
	this->tail = 0;
	this->count = 0;
	Mutex = xSemaphoreCreateMutex();
	for(int i=0;i<Worker;i++)
	{
		threadPool[i] = new WorkerThread(this);
	}
}
void Channel::startWorkers()
{
	for(int i=0;i<Worker;i++){
		threadPool[i]->start(&xHandle[i]);
	}
}
void Channel::putRequest(Request request)
{

       Dev_Manager::getInstance()->interface[UART]->Device_Write((void*)"PutRequest\r\n");
		while(count >= MAX_REQUEST)
		{
			//wait();
			taskYIELD();
		}
		requestQueue[tail] = request;
		tail = (tail + 1) % MAX_REQUEST;
		count++;			

	
}

Request Channel::takeRequest()
{

		Request req;
		Dev_Manager::getInstance()->interface[UART]->Device_Write((void*)"takeRequest\r\n");
		//vTaskDelay(500);
		while(count <= 0 )
		{
			taskYIELD();
		}
		req = requestQueue[head];
		Request *request = new Request();
		*request = requestQueue[head];			
		head = (head + 1) % MAX_REQUEST;
		count--;
	
		//notifyAll();
		return *request;

}
