/*
 * Request.h
 *
 * Created: 2016-12-01 오후 9:54:44
 *  Author: kimkisu
 */ 


#ifndef REQUEST_H_
#define REQUEST_H_
#include "FreeRTOS.h"
#include "task.h"



#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

class Request
{
private:
	int number;
public:
	Request();
	Request(const Request& copy);
	void setNumber(char number);
	void* operator new(size_t size);
	void operator delete(void* ptr);
	Request& operator=(const Request &ptr);
	void execute();
};



#endif /* REQUEST_H_ */