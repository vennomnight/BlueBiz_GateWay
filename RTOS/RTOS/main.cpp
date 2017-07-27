/*
 * RTOS.cpp
 *
 * Created: 2016-11-26 오후 2:50:59
 * Author : kimkisu
 */ 


#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>

#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "timeout.h"
#include "avr_compat.h"
#include "net.h"


#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "queue.h"
#include "Channel.h"
#include "Request.h"
#include "UartDriver.h"
#include "Dev_Manager.h"
#include "SerialBuffer.h"
#include "RS485Driver.h"
#include "per_sec_timer.h"
#include "Modbus_rtu.h"

//////////////////


#define DEBUG 0 //처음 부팅 디버깅용 
#define USE_ETH 1 //이더넷 사용
#define CHECK_ERROR 0 //에러 디버깅용
#define USE_SYSTEM_SEC 0

#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)


#if USE_ETH
#define	BUFFER_SIZE 512
#define MYWWWPORT 80
#define MYUDPPORT 9999
#endif


enum
{
	TEMP = 1,
	PRESSURE,
	COUNT,
	HOUR,
	MIN,
	SEC,
	ERROR_CNT,
	GOAL_CNT,
    MACHINE_STATES,
	#if USE_SYSTEM_SEC
		SYSTEM_SEC_CLOCK,
	#endif
	MAX_ENUM,		
};

typedef enum 
{
	NORMAL,
	MACHINE_BREAK,
	MACHINE_CHECK,
	COMPLETE_PRODUCTS,
	WAIT_MACHINE,
	WORKER_OUT,
	REST_TIME
}MACHINE_STATE_SUB_TITLE;


static void vTask1(void *pvParam);
static void vTask2(void *pvParam);
static void proc(void* pvParam);
static void proc1(void* pvParam);
static void proc2(void* pvParam);


/*
 * Idle hook is used to scheduler co-routines.
 */
extern "C"
{
	void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );
	void vApplicationIdleHook( void );

}
Dev_Manager *dev;

void *DataStruct[MAX] = {nullptr}; //Dev_Manager Enum Using
void Init_Dev();
void Uart_ISR(Dev_type Device,uint16_t Arg);
void RS485_ISR(Dev_type Device,uint16_t Arg);
void Timer_ISR(Dev_type Device,uint16_t Arg);


//Global Var

GetFunctionCode01 func01;
GetFunctionCode05 func05;
GetFunctionCode04 func04;
GetFunctionCode10 func10; 

RspExceptionCode exception;

ResponseFunctionCode10 rsp10;


//카운터 변수 
int count_number = 0;
//현재온도
int current_temp = 0;
//현재압력
int current_pressure = 0;

int mem4[MAX_ENUM] = {0};

int main( void )
{
	cli();  //인터럽트 금지 
	Init_Dev(); //dev 매니저 초기화
	
	
	
	dev->Open_Handle(UART0,Uart_ISR);  //드라이버 매니져에 인터럽트 루틴 등록
	dev->Open_Handle(RS485,RS485_ISR); //드라이버 매니져에 인터럽트 루틴 등록
	dev->Open_Handle(SEC_TIMER,Timer_ISR);
	SerialBuffer *sb = new SerialBuffer(dev,UART0); //링 버퍼 
	if(sb == nullptr)
	{
		#if DEBUG 
			dev->Writes(UART0,"RingBuffer Error\r\n");
		#endif
		exit(1);
	}
	else
	{
		#if DEBUG
			dev->Writes(UART0,"RingBuffer UART0 SUCCESS\r\n");
		#endif
	}
	SerialBuffer *sb1 = new SerialBuffer(dev,RS485); //링 버퍼 
	if(sb1 == nullptr)
	{
		#if DEBUG
			dev->Writes(UART0,"RingBuffer Error\r\n");
		#endif
		exit(1);
	}
	else
	{
		#if DEBUG
			dev->Writes(UART0,"RingBuffer UART1 SUCCESS\r\n");
		#endif
	}
	DataStruct[UART0] = sb;
	DataStruct[RS485] = sb1;
	for(uint8_t i=0;i<MAX;i++)
	{
		if(DataStruct[i] == nullptr)
		{
			#if DEBUG
				if(i == UART0)
					dev->Writes(UART0,"DataStruct UART0 Address Faile\r\n");
				else if(i == RS485)
					dev->Writes(UART0,"DataStruct UART1 Address Faile\r\n");
			#endif
		}
		else
		{
			#if DEBUG
				if(i == UART0)
					dev->Writes(UART0,"DataStruct UART0 Address SUCCESS\r\n");
				else if(i == RS485)
					dev->Writes(UART0,"DataStruct UART1 Address SUCCESS\r\n");
			#endif
		}
	}

	sei(); //인터럽트 사용 
	
	xTaskCreate(proc,                //테스크 실행할 함수 포인터
	"Task1",      //테스크 이름
	350,                   //스택의 크기
	sb,       // 테스크 매개 변수
	2,                     //테스크 우선 순위
	NULL                   //태스크 핸들
	);
	
		xTaskCreate(proc1,                //테스크 실행할 함수 포인터
		"Task2",      //테스크 이름
		350,                   //스택의 크기
		sb1,       // 테스크 매개 변수
		2,                     //테스크 우선 순위
		NULL                   //태스크 핸들
		);
		
		#if USE_ETH
				xTaskCreate(proc2,                //테스크 실행할 함수 포인터
				"Task3",      //테스크 이름
				240,                   //스택의 크기
				NULL,       // 테스크 매개 변수
				2,                     //테스크 우선 순위
				NULL                   //태스크 핸들
				);
		#endif
			
		
	
	vTaskStartScheduler();//스케줄러 실행 
	return 0;
}

void Init_Dev()
{
	dev = new Dev_Manager();
	dev->Register_Dev(new UartDriver,UART0);
	dev->Register_Dev(new RS485Driver,RS485);
	dev->Register_Dev(new Timer,SEC_TIMER);
	dev->Device_Init(UART0);
	dev->Device_Init(RS485);
	dev->Device_Init(SEC_TIMER);
	dev->Writes(UART0,"Uart Init SUCCESS boadrate 9600bps \r\n");
	dev->Writes(RS485,"RS485 Init SUCCESS boadrate 9600bps \r\n");
}

void Uart_ISR(Dev_type Device,uint16_t Arg)
{ 
	uint8_t data = Arg;
	SerialBuffer *sb = (SerialBuffer*)DataStruct[UART0];
	sb->Serialstore(data);

}
void RS485_ISR(Dev_type Device,uint16_t Arg)
{
	uint8_t data = Arg;
	SerialBuffer *sb = (SerialBuffer*)DataStruct[RS485];
	sb->Serialstore(data);
}
void Timer_ISR(Dev_type Device,uint16_t Arg)
{
	mem4[SEC]++;
	#if USE_SYSTEM_SEC
		mem4[SYSTEM_SEC_CLOCK]++;
	#endif
}
static void proc(void* pvParam) //터치패널 HMI RS232 쓰레드
{
	char read_Flag = 0;
	char function_code;
	char buf1[10];
	DDRG = 0xff;
	SerialBuffer *sb = (SerialBuffer*)pvParam;
		
	while(1)
	{
		if(mem4[SEC] >= 60)
		{
			mem4[SEC] = 0;
			mem4[MIN]++;
			if(mem4[MIN] >= 60)
			{
				mem4[MIN] = 0;
				mem4[HOUR]++;
			}
		}
		if(read_Flag == 0)
		{
			if(sb->SerialAvailable() >= 2)
			{
				for(int i=0;i<2;i++)
				{
					PORTG = 0xff;
					buf1[i] = sb->SerialRead();
				    PORTG = 0x00;
				}
				if(buf1[0] != 0x01)
				{
					read_Flag = 0;
				}
				if(buf1[1] == 0x01)
				{
					function_code = 0x01;
					read_Flag = 1;
				}
				else if(buf1[1] == 0x04)
				{
					function_code = 0x04;
					read_Flag = 1;
				}
				else if(buf1[1] == 0x05)
				{
					function_code = 0x05;                              
					read_Flag = 1;
				}
				else if(buf1[1] == 0x10)
				{
					function_code = 0x10;
					read_Flag = 1;
				}
				else
				{
					#if CHECK_ERROR
						mem4[ERROR_CNT]++;
					#endif
					GetExceptionCode(&exception,0x01,0x01);                                                                                                                                                                                                                  
					sb->SerialFlush();
					sb->SerialWrite((char*)&exception,sizeof(exception));
					read_Flag = 0;	
				}
			}
			
		}
		if(read_Flag == 1)
		{
			if(function_code == 0x01)  
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						PORTG = 0x00;
					}

				}
			}
			else if(function_code == 0x04) //Max1W 값 읽기
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						PORTG = 0x00;
					}
					GetFunc04Data(buf1,&func04,mem4);
					sb->SerialWrite((char*)&func04,sizeof(func04));
					read_Flag = 0;
				}
			}
			else if(function_code == 0x05) //터치 버튼 
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						PORTG = 0x00;
					}
					GetFunc05Data(buf1,&func05);
					sb->SerialWrite((char*)&func05,sizeof(func05));
					read_Flag = 0;
				}
			}
			else if(function_code == 0x10)  //3_MAX1W 입력 
			{
				if(sb->SerialAvailable() >= 9)
				{
					for(int i=2;i<11;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						//sb->SerialWrite(buf1[i]);
						PORTG = 0x00;
					}
					GetFucc10Data(buf1,&func10,mem4); //데이터 파싱
					ResponseFucc10Data(buf1,&rsp10); //리스폰스 데이터를 만듬.
					sb->SerialWrite((char*)&rsp10,sizeof(rsp10)); //리스폰스 데이터 쓰기.
					//sb->SerialWrite((char*)&rsp10,sizeof(rsp10));
					read_Flag = 0;
				}
						
			}
			
			/////////////////////////////////			
		}
	}
}
static void proc1(void* pvParam) 
{
	//UART1 
	SerialBuffer *sb = (SerialBuffer*)pvParam;
	SerialBuffer *sb1 = (SerialBuffer*)DataStruct[UART0];
	uint8_t proc1_buff[15] = {0};
	while(1)
	{
		if(sb->SerialAvailable() >= 15)
		{
			for(uint8_t i=0;i<15;i++)
			{
				proc1_buff[i] = sb->SerialRead();
			}
			if(proc1_buff[0] == 0x02)
			{
	
				count_number = ((0xff & proc1_buff[7]) << 8) | (0xff & proc1_buff[8]);
				current_temp = ((0xff & proc1_buff[9]) << 8) | (0xff & proc1_buff[10]);
				current_pressure = ((0xff & proc1_buff[11]) << 8) | (0xff & proc1_buff[12]);
				mem4[TEMP] = current_temp;
				mem4[COUNT] = count_number;
				mem4[PRESSURE] = current_pressure;
				if((mem4[GOAL_CNT] == mem4[COUNT])&& mem4[MACHINE_STATES] == NORMAL)
				{
					mem4[MACHINE_STATES] = COMPLETE_PRODUCTS;
				}
			}
			else
			{
				GetExceptionCode(&exception,0x01,0x06);
				sb->SerialWrite((char*)&exception,sizeof(exception)); //리스폰스 데이터 쓰기.
				cbi(UCSR1B,RXCIE0);
				sb->SerialFlush();
				sbi(UCSR1B,RXCIE0);
				
			}
		}
		
	}
}

#if USE_ETH
static void proc2(void* pvParam)
{
	static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x24};
	static uint8_t myip[4] = {192,168,0,108};
	static uint8_t buf[BUFFER_SIZE+1];
	uint16_t plen;
    char str[30];
	 enc28j60Init(mymac);
	 enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
	 vTaskDelay(10);
	 enc28j60PhyWrite(PHLCON,0x476);
	 vTaskDelay(20);
	 //init the ethernet/ip layer:
	 init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);

	 while(1)
	 {
		 plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
		 if(plen==0)
		 {
				continue;
		 }
		 if(eth_type_is_arp_and_my_ip(buf,plen))
		 {
			 make_arp_answer_from_request(buf);
			 continue;
		 }
		 if(eth_type_is_ip_and_my_ip(buf,plen)==0)
		 {
			 continue;
		 }
		 if(buf[IP_PROTO_P]==IP_PROTO_ICMP_V && buf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V)
		 {
			make_echo_reply_from_request(buf,plen);
			continue;
		 }
		 else//(mem4[SEC] % 5 == 0)
		 {
			 static int data[3] = {0};
			 data[0] = mem4[TEMP];
			 data[1] = mem4[COUNT];
			 data[2] = mem4[PRESSURE];
			 make_udp_reply_from_request(buf,(char*)&data,sizeof(data),MYUDPPORT);
		 }
	  }
}
#endif


void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

}





