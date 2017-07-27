/*
 * Modbus_rtu.h
 *
 * Created: 2017-01-27 오후 5:27:17
 *  Author: kimkisu
 */ 


#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_

typedef struct RspExceptionCode
{
	char Address;
	char FunctionCode;
	char ExceptionCode;
	short CRC;
}__attribute__((packed)) RspExceptionCode;


typedef struct GetFunctionCode01
{
	char Address;
	char FunctionCode;
	char ByteCount;
	char OutPutStatus1;
	char OutPutStatus2;
	int CRC;
}__attribute__((packed)) GetFunctionCode01;

typedef struct GetFunctionCode04
{
	char Address;
	char FunctionCode;
	char ByteCount;
	char InputRegHi;
	char InputRegLo;
	int CRC;
}__attribute__((packed)) GetFunctionCode04;

typedef struct GetFunctionCode05
{
	char Address;
	char FunctionCode;
	char OutputAddressHi;
	char OutputAddressLo;
	char OutputValueHi;
	char OutputValueLo;
	int CRC;
}__attribute__((packed)) GetFunctionCode05;

typedef struct GetFunctionCode10
{
	char address;
	char functionCode;
	char startingAddressHi;
	char startingAddressLo;
	char quantityOfRegistersHi;
	char quantityOfRegistersLo;
	char byteCount;
	unsigned char registerValueHi;
	unsigned char registerValueLo;
	int CRC;
}__attribute__((packed)) GetFunctionCode10;

typedef struct ResponseFunctionCode10
{
	char address;
	char functionCode;
	char startingAddressHi;
	char startingAddressLo;
	char quantityOfRegistersHi;
	char quantityOfRegistersLo;
	int CRC;
}__attribute__((packed)) ResponseFunctionCode10;




///RS485
typedef struct InputOutput8Byte //8바이트 모드버스 프로토콜
{
	unsigned char Address;
	unsigned char FunctionCode;
	unsigned int writeData;
	unsigned int NumberOfData;
	int CRC;
}__attribute__((packed)) InputOutput8Byte;

typedef struct GetData  //데이터 받는 구조체
{
	unsigned char Address;
	unsigned char FunctionCode;
	unsigned char ByteLength;
	unsigned int CurrentPv;
	int CRC;
}__attribute__((packed)) GetData;

void getFunction3Data(char* buf,GetData* struct_File);// 온도값 가져오기 
void Function03Write(char Address,int Dregister,char num,InputOutput8Byte* struct_File); // 모드버스 RTU 03 모드 주소 설정 레지스터 설정 함수
void Function06Write(char Address,int Dregister,int TempVal,InputOutput8Byte* struct_File);


///


unsigned short CRC16(unsigned char *puchMsg, int usDataLen);
int to_little(int bit16);
int to_big(int bit16);
void GetExceptionCode(RspExceptionCode* Exception,char adr,char Exception_code);
void GetFunc01Data(char* buf,GetFunctionCode01* function01,char* func1mem);
void GetFunc04Data(char* buf,GetFunctionCode04* function04,int* func4mem);
void GetFunc05Data(char* buf,GetFunctionCode05* function05);
void GetFucc10Data(char* buf,GetFunctionCode10* function10,int* func4mem);
void ResponseFucc10Data(char* buf,ResponseFunctionCode10 *rfunction10);






#endif /* MODBUS_RTU_H_ */