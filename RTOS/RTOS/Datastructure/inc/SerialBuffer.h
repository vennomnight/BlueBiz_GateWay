/*
 * UartBuffer.h
 *
 * Created: 2016-12-15 오후 8:26:52
 *  Author: kimkisu
 */ 


#ifndef UARTBUFFER_H_
#define UARTBUFFER_H_
#include <avr/io.h>
#include <avr/interrupt.h>

#define RX_BUFFER_SIZE 128
#include "FreeRTOS.h"
#include "task.h"
#include "Dev_Manager.h"

typedef struct RingBuffer //내부 버퍼 큐 구조체
{
	char internalBuffer[RX_BUFFER_SIZE];
	unsigned int head;
	unsigned int tail;
}__attribute__((packed)) RingBuffer;


class Dev_Manager;
class SerialBuffer
{
private:
	RingBuffer ib;
	char num;
	Dev_type type;
	Dev_Manager* dev;
public:
	SerialBuffer(Dev_Manager* _dev,Dev_type _type);
	~SerialBuffer() = default;
	void* operator new(size_t size);
	void operator delete(void* ptr);
	void Serialstore(char data);// 내부 버퍼에 시스쳄 버퍼 저장 UDR에 데이터가 들어오면
	void SerialWrite(char data);// 한 바이트 쓰기 함수
	void SerialWrite(const char* buf, int buf_size); // *주소 바이트 쓰기 함수
	void SerialWrite(const char* buf); //문자열을 보냄.
	void SerialWrite(int Integer); //정수 표시
	char SerialRead(void); //데이터 읽기
	unsigned char SerialAvailable(void); // 가용 자원수 확인
	void SerialFlush(void); // 버퍼를 비운다.

	
};
#endif /* UARTBUFFER_H_ */