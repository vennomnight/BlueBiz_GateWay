#include "ModBus.h"
volatile char data = 0;
InputOutput11Byte wFunc16;
InputOutput8Byte gFunc16;
InputOutput8Byte wFunc3;

GetData getData;
RingBuffer ib;

volatile int num;
unsigned short CRC16(unsigned char *puchMsg, int usDataLen) {
  int register i;
  unsigned short crc, flag;
  crc = 0xffff;
  while (usDataLen--) {
    crc ^= *puchMsg++;
    for (i = 0; i<8; i++) {
      flag = crc & 0x0001;
      crc >>= 1;
      if (flag) crc ^= 0xA001;
    }
  }
  return crc;
}
void SerialEnd(void)
{
  UCSR0B = 0x00; 
}
void SerialRestart(void)
{
  UCSR0B = 0x98;
}
void SerialFlush(void)
{
  UDR0 = 0x00;
  ib.head = 0;
  ib.tail = 0;
}
int to_little(int bit16)
{
  unsigned char Byte[2];
  int ret;
  Byte[0] = (unsigned char)((bit16 >> 0) & 0xff);
  Byte[1] = (unsigned char)((bit16 >> 8) & 0xff);
  ret = (((int)Byte[0] << 0 ) | ((int)Byte[1] << 8));
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
char* Function16Write(char Address,int Dregister,int Data)
{
  unsigned char val[8] = {0};
  register unsigned int i = 0;
  unsigned short crc16;
  unsigned int BUFSIZE = sizeof(InputOutput11Byte);
  wFunc16.Address = Address;
  wFunc16.FunctionCode = 16;
  wFunc16.Dregister = to_big(Dregister);
  wFunc16.Number = to_big(1);
  wFunc16.ByteLength = 2;
  wFunc16.writeData0 = to_big(Data);
  crc16 = CRC16((unsigned char*)&wFunc16,BUFSIZE-2);
  wFunc16.CRC = crc16;
  return (char*)&wFunc16;
}
char* Function03Write(char Address,int Dregister)
{
  unsigned char val[8] = {0};
  unsigned short crc16;
  unsigned int BUFSIZE = sizeof(InputOutput8Byte);
  wFunc3.Address = Address;
  wFunc3.FunctionCode = 3;
  wFunc3.writeData = to_big(Dregister);
  wFunc3.NumberOfData = to_big(1);
  crc16 = CRC16((unsigned char*)&wFunc3,BUFSIZE-2);
  wFunc3.CRC = crc16;
  return (char*)&wFunc3;
}
int getFunction3Data(void)
{
  unsigned char val[7] = {0};
  unsigned char i = 0;
  while(i < 7 )
  {
   if(SerialAvailable() == 7);
   {
       val[i] = SerialRead();
       i++;
    }
  }
  getData = *(GetData*)val;
  getData.CurrentPv = to_big(getData.CurrentPv);
  return getData.CurrentPv;
}
void getFunction16Data(void)
{
  unsigned char val[8] = {0};
  unsigned char i = 0;
  while(i < 8)
  {
    if(SerialAvailable())
    {
       val[i] = SerialRead();
       i++;
    }
  }
  gFunc16 = *(InputOutput8Byte*)val;
  gFunc16.writeData = to_big(gFunc16.writeData);
  gFunc16.NumberOfData = to_big(gFunc16.NumberOfData);
  gFunc16.CRC = to_big(gFunc16.CRC);
}
ISR(USART_RX_vect)
{
 data = UDR0;
 Serialstore(data);
}
void Serialinit(void)
{
	asm volatile
		(
			"ldi r16,0x00 \n\t"
			"sts 0xc5,r16 \n\t"
			"ldi r17,0x67 \n\t"
			"sts 0xc4,r17 \n\t"
			"ldi r18,0x98 \n\t"
			"sts 0xc1,r18 \n\t"
			"ldi r19,0x80 \n\t"
			"sts 0x3f,r19 \n\t"
			);
      UCSR0C = 0x06;
}
void Serialstore(char data)
{
	unsigned int i = (ib.head + 1) % RX_BUFFER_SIZE;
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
void SerialWrite(unsigned char data)
{
	while (!(UCSR0A & 0x20));
  UDR0 = data;
}
void SerialWrites(const char* buf, int buf_size)
{
	volatile int i;
	for (i = 0; i<buf_size; i++)
	{
		SerialWrite(*(buf + i));
	}
}
int SerialRead(void)
{
	if (ib.head == ib.tail)
	{
		return -1;
	}
	else
	{
		char data = ib.internalBuffer[ib.tail + 1];
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
int SerialAvailable(void)
{
	return (unsigned int)(RX_BUFFER_SIZE + (ib.head - ib.tail)) % RX_BUFFER_SIZE;
}
