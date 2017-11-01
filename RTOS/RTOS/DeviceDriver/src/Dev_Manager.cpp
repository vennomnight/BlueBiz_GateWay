/*
 * Dev_Manager.cpp
 *
 * Created: 2016-12-02 오후 9:21:14
 *  Author: kimkisu
 */ 
#include "Dev_Manager.h"
#include "DeviceDriverInterface.h"

Dev_Manager* Dev_Manager::inst = nullptr;
Dev_Manager::Dev_Manager()
{
	inst = this;
}
Dev_Manager::~Dev_Manager()
{
	inst = nullptr;
	for(uint8_t i =0; i< MAX ; i++)
	{
		interface[i] = nullptr;
	}
}
Dev_Manager* const Dev_Manager::getInstance()
{
	if (inst == nullptr)
		inst = new Dev_Manager();
	return inst;
}
void Dev_Manager::Register_Dev(DeviceDriveInterFace * const _interface,Dev_type Dev)
{
	interface[Dev] = _interface;
}
void* Dev_Manager::operator new(size_t size)
{
	return malloc(size);
}
void Dev_Manager::operator delete(void* ptr)
{
	free(ptr);
}
void Dev_Manager::Write(Dev_type Device,char data)
{
	if(interface[Device])
	{
		interface[Device]->Device_Write(data);
	}
}
void Dev_Manager::Writes(Dev_type Device,const char* data)
{
	if(interface[Device])
	{
		interface[Device]->Device_Writes(data);
	}
}
void Dev_Manager::Device_Init(Dev_type Device)
{
	if(interface[Device])
	{
		interface[Device]->Device_Init();
	}
}
void Dev_Manager::Open_Handle(Dev_type Device,const ISR_Handle _isr_handle)
{
	if(interface[Device])
	{
		isr_handle[Device] = _isr_handle;
	}
}
char Dev_Manager::Driver_Check(Dev_type Device)
{
	if(interface[Device])
	{
		return 1;
	}
	else
		return 0;
}
void Dev_Manager::Close_Handle(Dev_type Device)
{
	if(interface[Device])
	{
		isr_handle[Device] = nullptr;
	}
}
DeviceDriveInterFace* const Dev_Manager::getInterfaceAddr(Dev_type Device) const
{
	if(interface[Device])
	{
		return interface[Device];
	}
	else
		return nullptr;
}
ISR(USART0_RX_vect)
{
	Dev_Manager::getInstance()->isr_handle[UART0](UART0,UDR0);
}
ISR(USART1_RX_vect)
{
	Dev_Manager::getInstance()->isr_handle[RS485](RS485,UDR1);
}
ISR(TIMER3_COMPC_vect)
{
	Dev_Manager::getInstance()->isr_handle[SEC_TIMER](SEC_TIMER,NULL);
}
ISR(INT0_vect)
{
	Dev_Manager::getInstance()->isr_handle[COUNT_SENSOR](COUNT_SENSOR,NULL);
}
ISR(ADC_vect)
{	
	Dev_Manager::getInstance()->isr_handle[_ADC](_ADC,ADCL+((uint16_t)ADCH << 8));
}
