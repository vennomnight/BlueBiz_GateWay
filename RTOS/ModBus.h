/*
시스템 알고리즘 
환영큐 사용 

//Function03Write 8Byte InputOutput8Byte
//Function16Write 11Byte sizeof(InputOutput11Byte)
*/
#ifndef MODBUS_H
#define MODBUS_H
#include "Arduino.h"
#endif
#define RX_BUFFER_SIZE 128
unsigned short CRC16(unsigned char *puchMsg, int usDataLen); // CRC 체크섬 함수
int to_little(int bit16); // 리트엔디안 변환함수
int to_big(int bit16);   // 빅엔디안 변환 함수 
char* Function16Write(char Address,int Dregister,int Data); //모드버스 RTU 16모드 주소설정 레지스터 데이타 설정 함수
char* Function03Write(char Address,int Dregister); // 모드버스 RTU 03 모드 주소 설정 레지스터 설정 함수
ISR(USART_RX_vect); // 시리얼통신 인터럽트 벡터 함수
void Serialinit(void); // 시리얼통신 초기화 9600
void Serialstore(char data); // 내부 버퍼에 시스템 버퍼 내용을 저장 
void SerialWrite(unsigned char data); //한 바이트 쓰기 함수
void SerialWrites(const char* buf, int buf_size); //*주소 바이트 쓰기 주소와 길이 
int SerialRead(void); // 데이터 읽기 
int SerialAvailable(void); // 가용 자원수 확인 함수 
void SerialFlush(void);  //시스템 버퍼를 비운다.
int getFunction3Data(void); // 온도값 가져오기 
void getFunction16Data(void); // 16번 기능의 대한 수신 버퍼 
void SerialEnd(void);
void SerialRestart(void);
typedef struct InputOutput11Byte //11바이트 모드버스 프로토콜 
{
	unsigned char Address;
	unsigned char FunctionCode;
	unsigned int Dregister;
	unsigned int Number;
	unsigned char ByteLength;
	unsigned int writeData0;
	unsigned int CRC;
}__attribute__((packed)) InputOutput11Byte;
typedef struct InputOutput8Byte //8바이트 모드버스 프로토콜
{
	unsigned char Address;
	unsigned char FunctionCode;
	unsigned int writeData;
	unsigned int NumberOfData;
	unsigned int CRC;
}__attribute__((packed)) InputOutput8Byte;
typedef struct GetData  //데이터 받는 구조체
{
	unsigned char Address;
	unsigned char FunctionCode;
	unsigned char ByteLength;
	unsigned int CurrentPv;
	unsigned int CRC;
}__attribute__((packed)) GetData;
typedef struct RingBuffer //내부버퍼 큐 구조체
{
	char internalBuffer[RX_BUFFER_SIZE];
	unsigned int head = 0;
	unsigned int tail = 0;
}__attribute__((packed)) RingBuffer;
