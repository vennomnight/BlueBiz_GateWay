/*
 * Modbus_rtu.cpp
 *
 * Created: 2017-01-27 오후 5:27:33
 *  Author: kimkisu
 */ 
#include "Modbus_rtu.h"

unsigned short CRC16(unsigned char *puchMsg, int usDataLen)
{
	int register i;
	unsigned short crc, flag;
	crc = 0xffff;
	while (usDataLen--)
	{
		crc ^= *puchMsg++;
		for (i = 0; i<8; i++)
		{
			flag = crc & 0x0001;
			crc >>= 1;
			if (flag) crc ^= 0xA001;
		}
	}
	return crc;
}
int to_little(int bit16) //하위 바이트를 상위 바이트로 바꿈.
{
	unsigned char Byte[2];
	int ret;
	Byte[0] = (unsigned char)((bit16 >> 0) & 0xff);
	Byte[1] = (unsigned char)((bit16 >> 8) & 0xff);
	ret = (((int)Byte[0] << 0) | ((int)Byte[1] << 8));
	return ret;
}
int to_big(int bit16)
{
	unsigned char Byte[2];
	int ret;
	Byte[0] = (unsigned char)((bit16 >> 0) & 0xff);
	Byte[1] = (unsigned char)((bit16 >> 8) & 0xff);
	ret = ((int)Byte[0] << 8) | ((int)Byte[1] << 0);
	return ret;
}
void GetFunc01Data(char* buf,GetFunctionCode01* function01,char* func1mem)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(GetFunctionCode01);
	function01->Address = *(buf + 0);
	function01->FunctionCode = *(buf + 1);
	function01->ByteCount = 0x02;
	function01->OutPutStatus1 = func1mem[*(buf + 3)];
	function01->OutPutStatus2 = func1mem[(*(buf + 3)) + 1];
	crc16 = CRC16((unsigned char*)function01, BUFSIZE - 2);
	function01->CRC = crc16;
}
void GetFunc04Data(char* buf,GetFunctionCode04* function04,int* func4mem)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(GetFunctionCode04);
	char address = 0;
	function04->Address = *(buf + 0);
	function04->FunctionCode = *(buf + 1);
	address = *(buf + 3);
	function04->ByteCount = 0x02;
	function04->InputRegHi = (func4mem[address] >> 8);
	function04->InputRegLo = func4mem[address];
	crc16 = CRC16((unsigned char*)function04, BUFSIZE - 2);
	function04->CRC = crc16;
}
void GetFunc05Data(char* buf,GetFunctionCode05* function05)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(GetFunctionCode05);
	function05->Address = *(buf + 0);
	function05->FunctionCode = *(buf + 1);
	function05->OutputAddressHi = *(buf + 2);
	function05->OutputAddressLo = *(buf + 3);
	function05->OutputValueHi = *(buf + 4);
	function05->OutputValueLo = *(buf + 5);
	crc16 = CRC16((unsigned char*)function05, BUFSIZE - 2);
	function05->CRC = crc16;//to_little(crc16);
}
void GetFucc10Data(char* buf,GetFunctionCode10* function10,int* func4mem)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(GetFunctionCode10);
	function10->address = *(buf + 0);
	function10->functionCode = *(buf + 1);
	function10->startingAddressHi = *(buf + 2);
	function10->startingAddressLo = *(buf + 3);
	function10->quantityOfRegistersHi = *(buf + 4);
	function10->quantityOfRegistersLo = *(buf + 5);
	function10->byteCount = *(buf + 6);
	function10->registerValueHi = *(buf + 7);
	function10->registerValueLo = *(buf + 8);
	func4mem[function10->startingAddressLo] = ((function10->registerValueHi << 8) | (function10->registerValueLo));
	crc16 = CRC16((unsigned char*)function10, BUFSIZE - 2);
	function10->CRC = crc16;
}
void ResponseFucc10Data(char* buf,ResponseFunctionCode10 *rfunction10)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(ResponseFunctionCode10);
	rfunction10->address = *(buf + 0);
	rfunction10->functionCode = *(buf + 1);
	rfunction10->startingAddressHi = *(buf + 2);
	rfunction10->startingAddressLo = *(buf + 3);
	rfunction10->quantityOfRegistersHi = *(buf + 4);
	rfunction10->quantityOfRegistersLo = *(buf + 5);
	crc16 = CRC16((unsigned char*)rfunction10, BUFSIZE - 2);
	rfunction10->CRC = crc16;
}
void GetExceptionCode(RspExceptionCode* Exception,char adr,char Exception_code)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(RspExceptionCode);
	Exception->Address = adr;
	Exception->FunctionCode = 0x81;
	Exception->ExceptionCode = Exception_code;
	crc16 = CRC16((unsigned char*)Exception, BUFSIZE - 2);
	Exception->CRC = crc16;
}


//////////////////////////////
void getFunction3Data(char* buf,GetData* struct_File)
{
	struct_File->Address = buf[0];
	struct_File->FunctionCode = buf[1];
	struct_File->ByteLength = buf[2];
	struct_File->CurrentPv = 0xff00 & (buf[3] << 8);
	struct_File->CurrentPv |= buf[4];
}
void Function03Write(char Address,int Dregister,char num,InputOutput8Byte* struct_File)
{
	unsigned short crc16;
	unsigned int BUFSIZE = sizeof(InputOutput8Byte);
	struct_File->Address = Address;
	struct_File->FunctionCode = 3;
	struct_File->writeData = to_big(Dregister);
	struct_File->NumberOfData = to_big(num);
	crc16 = CRC16((unsigned char*)struct_File,BUFSIZE-2);
	struct_File->CRC = crc16;
}
void Function06Write(char Address,int Dregister,int TempVal,InputOutput8Byte* struct_File)
{
		unsigned short crc16;
		unsigned int BUFSIZE = sizeof(InputOutput8Byte);
		struct_File->Address = Address;
		struct_File->FunctionCode = 0x06;
		struct_File->writeData = to_big(Dregister);
		struct_File->NumberOfData = to_big(TempVal);
		crc16 = CRC16((unsigned char*)struct_File,BUFSIZE-2);
		struct_File->CRC = crc16;
}