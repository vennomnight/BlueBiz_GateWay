/*
 * Request.cpp
 *
 * Created: 2016-12-01 오후 9:56:05
 *  Author: kimkisu
 */ 
#include "Request.h"
#include "UartDriver.h"
#include "Dev_Manager.h"
Request::Request(){};
Request::Request(const Request& copy)
{
	this->number = copy.number;
}
void Request::setNumber(char number)
{
	this->number = number;
}
void* Request::operator new(size_t size)
{
	return malloc(size);
}
void Request::operator delete(void* ptr)
{
	free(ptr);
}
Request& Request::operator=(const Request &ptr)
{
	
	this->number = ptr.number;
	return *this;
}
void Request::execute()
{
	Dev_Manager::getInstance()->interface[UART]->Device_Write((void*)"====excute \r\n");
}