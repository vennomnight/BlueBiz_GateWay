/*
 * DeviceDriverInterface.h
 *
 * Created: 2016-12-02 오후 9:17:51
 *  Author: kimkisu
 */ 


#ifndef DEVICEDRIVERINTERFACE_H_
#define DEVICEDRIVERINTERFACE_H_

class DeviceDriveInterFace
{
	public:
	virtual void Device_Init() = 0;
	virtual char Device_Read() const{return 0;}
	virtual void Device_Writes(const char* data){}
	virtual void Device_Write(char data){}
	virtual void Stop_Device(void){}
	virtual void Start_Device(void){}
};



#endif /* DEVICEDRIVERINTERFACE_H_ */