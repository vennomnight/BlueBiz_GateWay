/*
 * UartBuffer.cpp
 *
 * Created: 2016-12-15 오후 8:37:34
 *  Author: kimkisu
 */ 
#include <stdio.h>
#include "SerialBuffer.h"
#include "Dev_Manager.h"
const unsigned char HexaString[] = "0123456789ABCDEF";

SerialBuffer::SerialBuffer(Dev_Manager* _dev,Dev_type _type)
{
	this->dev = _dev;
	if(dev->Driver_Check(_type))
	{
		this->type = _type;
		this->num = 0;
		this->ib.head = 0;
		this->ib.tail = 0;
	}
	else
		this->dev = nullptr;

}
void* SerialBuffer::operator new(size_t size)
{
	return malloc(size);
}
void SerialBuffer::operator delete(void* ptr)
{
	free(ptr);
}
void SerialBuffer::Serialstore(char data)// 내부 버퍼에 시스쳄 버퍼 저장 UDR에 데이터가 들어오면
{
	unsigned char i = (ib.head + 1) % RX_BUFFER_SIZE;
	if (i == 0)
	{
		i = 1;
		ib.head = 0;
	}
	if (i != ib.tail)
	{
		ib.internalBuffer[ib.head + 1] = data;
		ib.head = i;
		num++;
	}
}
void SerialBuffer::SerialWrite(char data)// 한 바이트 쓰기 함수
{
	if(dev->getInterfaceAddr(this->type))
	{
		dev->Write(this->type,data);
	}
}
void SerialBuffer::SerialWrite(const char* buf, int buf_size) // *주소 바이트 쓰기 함수
{
	if(dev->getInterfaceAddr(this->type))
	{
		for(char i=0;i<buf_size;i++)
		{
			dev->Write(this->type,*(buf+i));
		}
	}
}
void SerialBuffer::SerialWrite(const char* buf) //문자열을 보냄.
{
	while(*buf)
	{
		dev->Write(this->type,*buf++);
	}
}
void SerialBuffer::SerialWrite(int Integer) //정수 표시
{
	unsigned char Temp;
	
	Temp = Integer / 10000 % 10;	
	Temp = HexaString[Temp];

	dev->Write(this->type,Temp);

	Temp = Integer / 1000 % 10;	
	Temp = HexaString[Temp];
	
	dev->Write(this->type,Temp);

	
	Temp = Integer / 100 % 10;	
	Temp = HexaString[Temp];
	dev->Write(this->type,Temp);

	
	Temp = Integer / 10 % 10;	
	Temp = HexaString[Temp];
	dev->Write(this->type,Temp);

	
	Temp = Integer % 10;	
	Temp = HexaString[Temp];
	dev->Write(this->type,Temp);

	
}
char SerialBuffer::SerialRead(void) //데이터 읽기
{
	if (ib.head == ib.tail)
	{
		return -1;
	}
	else
	{
		unsigned char data = ib.internalBuffer[ib.tail + 1];
		ib.tail = (ib.tail + 1) % RX_BUFFER_SIZE;
		if (ib.tail == num)
		{
			ib.tail = 0;
			ib.head = 0;
			num = 0;
		}
		return data;
	}
}
unsigned char SerialBuffer::SerialAvailable(void) // 가용 자원수 확인
{
	return (unsigned char)(RX_BUFFER_SIZE + (ib.head - ib.tail)) % RX_BUFFER_SIZE;
}

void SerialBuffer::SerialFlush(void)// 버퍼를 비운다.
{
	ib.tail = 0;
	ib.head = 0;
	num = 0;
} 
