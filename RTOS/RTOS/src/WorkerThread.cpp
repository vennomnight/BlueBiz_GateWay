/*
 * WorkerThread.cpp
 *
 * Created: 2016-12-01 오후 10:07:11
 *  Author: kimkisu
 */ 

#include "WorkerThread.h"
#include "Request.h"	
#include "Channel.h"
#include "UartDriver.h"
#include "Dev_Manager.h"
void ThreadCreate(WorkerThread* ptr,Channel *channel ,TaskHandle_t *xHandle)
{
	static char i=0;
	xTaskCreate(ptr->run,                //테스크 실행할 함수 포인터
		(char*)i,      //테스크 이름
		200,                   //스택의 크기
		channel,       // 테스크 매개 변수
		2,                     //테스크 우선 순위
		xHandle                  //태스크 핸들
		);
		i++;
}
WorkerThread::WorkerThread(){}
WorkerThread::WorkerThread(Channel *channel)
{
	this->channel = channel;
}
void* WorkerThread::operator new(size_t size)
{
	return malloc(size);
}
void WorkerThread::start(TaskHandle_t *xHandle)
{
	th = ThreadCreate;
	th(this,this->channel,xHandle);

}
void WorkerThread::run(void* pvParam)
{
	Channel *p = (Channel*)pvParam;
	while(p->ProcCount() > 0)
	{
		Request request(p->takeRequest());
		request.execute();
		taskYIELD();
	}
}
