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
#include <avr/eeprom.h>


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
#include "Count_Sensor.h"
#include "Alarm.h"
#include "Timer_Alarm.h"
//////////////////


const char debug = 0; //처음 부팅 디버깅용 
#define USE_ETH 1 //이더넷 사용
#define CHECK_ERROR 0 //에러 디버깅용
#define USE_SYSTEM_SEC 0 //총 시스템 초
//#define EXTERNAL_COUNT_SENSOR 1 // 외부 센서 사용시


//#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
//#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)


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
	UDP_DATA0, //start
	UDP_DATA1,
	UDP_DATA2,
	UDP_DATA3,
	UDP_DATA4,
	UDP_DATA5, //serial
	UDP_DATA6, //start
	UDP_DATA7,
	UDP_DATA8,
	UDP_DATA9,
	UDP_DATA10,
	UDP_DATA11,
	UDP_DATA12,
	UDP_DATA13,  // product
	UDP_DATA14,  //cnt
	UDP_DATA15,  //cnt
    WARNING_LOW,
	WARNING_HIGH,
	TARGET_GOAL_CNT,
	TARGET_MIN,
	TARGET_MAX,
	TARGET_CMP,
	IPV4_0,
	IPV4_1,
	IPV4_2,
	IPV4_3,
	TARGET_COUNT_SENSOR,  //0 PLC 1 LOCAL
	TEST1,
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
void Count_Sensor_ISR(Dev_type Device,uint16_t Arg);

void Set_Alarm();

static void System_Init();

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

int mem4[MAX_ENUM] = {0};  //function 10 메모리 공간
char cmp_mem[4] = {0};  //IP 주소 비교 메모리 

uint8_t chatter_flag = 0; //채터링 방지 플래그 변수
//uint8_t use_external_count_sensor = 0; // 0 PLC  1 EXTERNAL COUTN SENSOR

int main( void )
{
	System_Init();
	cli();  //인터럽트 금지 
	Init_Dev(); //dev 매니저 초기화
	
	
	dev->Open_Handle(UART0,Uart_ISR);  //드라이버 매니져에 인터럽트 루틴 등록
	dev->Open_Handle(RS485,RS485_ISR); //드라이버 매니져에 인터럽트 루틴 등록
	dev->Open_Handle(SEC_TIMER,Timer_ISR);
	dev->Open_Handle(COUNT_SENSOR,Count_Sensor_ISR);
	SerialBuffer *sb = new SerialBuffer(dev,UART0); //링 버퍼 
	if(sb == nullptr)
	{
		if(debug)
		{ 
			dev->Writes(UART0,"RingBuffer Error\r\n");
		}
		exit(1);
	}
	else
	{
		if(debug)
		{
			dev->Writes(UART0,"RingBuffer UART0 SUCCESS\r\n");
		}
	}
	SerialBuffer *sb1 = new SerialBuffer(dev,RS485); //링 버퍼 
	if(sb1 == nullptr)
	{
		if(debug)
		{
			dev->Writes(UART0,"RingBuffer Error\r\n");
		}
		exit(1);
	}
	else
	{
		if(debug)
		{
			dev->Writes(UART0,"RingBuffer UART1 SUCCESS\r\n");
		}
	}
	DataStruct[UART0] = sb;
	DataStruct[RS485] = sb1;
	for(uint8_t i=0;i<MAX;i++)
	{
		if(DataStruct[i] == nullptr)
		{
			if(debug)
			{
				if(i == UART0)
					dev->Writes(UART0,"DataStruct UART0 Address Faile\r\n");
				else if(i == RS485)
					dev->Writes(UART0,"DataStruct UART1 Address Faile\r\n");
			}
		}
		else
		{
			if(debug)
			{
				if(i == UART0)
					dev->Writes(UART0,"DataStruct UART0 Address SUCCESS\r\n");
				else if(i == RS485)
					dev->Writes(UART0,"DataStruct UART1 Address SUCCESS\r\n");
			}
		}
	}
	Alarm_Init(); //알람 초기화
	////Timer_Alarm al;
	
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
				2,                     //테스크 우선 순위0.
				NULL                   //태스크 핸들
				);
		#endif
			
		
	
	vTaskStartScheduler();//스케줄러 실행 
	return 0;
}
static void System_Init()
{
	mem4[IPV4_0] = eeprom_read_byte((const uint8_t*)0);  //read ip address
	mem4[IPV4_1] = eeprom_read_byte((const uint8_t*)1);
	mem4[IPV4_2] = eeprom_read_byte((const uint8_t*)2);
	mem4[IPV4_3] = eeprom_read_byte((const uint8_t*)3);
	cmp_mem[0] = mem4[IPV4_0];
	cmp_mem[1] = mem4[IPV4_1];
	cmp_mem[2] = mem4[IPV4_2];
	cmp_mem[3] = mem4[IPV4_3];
	
}
void Init_Dev()
{
	dev = new Dev_Manager();
	dev->Register_Dev(new UartDriver,UART0);
	dev->Register_Dev(new RS485Driver,RS485);
	dev->Register_Dev(new Timer,SEC_TIMER);
	dev->Register_Dev(new Count_Sensor,COUNT_SENSOR);
	dev->Device_Init(UART0);
	dev->Device_Init(RS485);
	dev->Device_Init(SEC_TIMER);
	dev->Device_Init(COUNT_SENSOR);
	dev->Writes(UART0,"Uart Init SUCCESS boadrate 9600bps \r\n");
	dev->Writes(RS485,"RS485 Init SUCCESS boadrate 9600bps \r\n");
}

void Uart_ISR(Dev_type Device,uint16_t Arg)
{ 
	uint8_t data = Arg;
	SerialBuffer *sb = (SerialBuffer*)DataStruct[UART0];
	sb->Serialstore(data);
	sbi(PORTB,5);

}
void RS485_ISR(Dev_type Device,uint16_t Arg)
{
	uint8_t data = Arg;
	SerialBuffer *sb = (SerialBuffer*)DataStruct[RS485];
	sb->Serialstore(data);
	sbi(PORTB,6);
}
void Timer_ISR(Dev_type Device,uint16_t Arg)
{
	mem4[SEC]++;
	cbi(PORTB,5);
	cbi(PORTB,6);
	#if USE_SYSTEM_SEC
		mem4[SYSTEM_SEC_CLOCK]++;
	#endif
}
void Count_Sensor_ISR(Dev_type Device,uint16_t Arg)
{
	Alarm_Open(ALARM0,20,Set_Alarm);
}
void Set_Alarm()
{
	if(chatter_flag == 0)
	{
		chatter_flag = 1;
	}
}
static void proc(void* pvParam) //터치패널 HMI RS232 쓰레드
{
	char read_Flag = 0;
	char function_code;
	char buf1[10];
	//SerialBuffer *sb = (SerialBuffer*)pvParam;
	SerialBuffer *sb = static_cast<SerialBuffer*>(pvParam);	
	register uint16_t i;
	while(1)
	{
		//mem4[COUNT] = PIND;
		if(PIND == 0xfe && chatter_flag == 1)//
		{
			mem4[COUNT]++;                // Remove Chattering 
			chatter_flag = 2;
		}
		if(PIND == 0xff && chatter_flag == 2)//
		{
			vTaskDelay(20);
			chatter_flag = 0;
		}
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
				for(i=0;i<2;i++)
				{
					buf1[i] = sb->SerialRead();
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
					cbi(PORTB,7);
					cbi(UCSR0B,RXCIE0);                                                                                                                                                                                                                
					sb->SerialFlush();
					sbi(UCSR0B,RXCIE0);
					sb->SerialWrite((char*)&exception,sizeof(exception));
					sbi(PORTB,7);
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
					for(i=2;i<8;i++)
					{
						buf1[i] = sb->SerialRead();
					}

				}
			}
			else if(function_code == 0x04) //Max1W 값 읽기
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(i=2;i<8;i++)
					{
						buf1[i] = sb->SerialRead();
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
					for(i=2;i<8;i++)
					{
						buf1[i] = sb->SerialRead();
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
					for(i=2;i<11;i++)
					{
						buf1[i] = sb->SerialRead();
					}
					GetFucc10Data(buf1,&func10,mem4); //데이터 파싱
					ResponseFucc10Data(buf1,&rsp10); //리스폰스 데이터를 만듬.
					sb->SerialWrite((char*)&rsp10,sizeof(rsp10)); //리스폰스 데이터 쓰기.
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
	SerialBuffer *sb = static_cast<SerialBuffer*>(pvParam);
	uint8_t proc1_buff[15] = {0};
	while(1)
	{
	   if(mem4[TARGET_CMP] != mem4[GOAL_CNT])
	   {
		   mem4[TARGET_MAX] = mem4[GOAL_CNT];
		   mem4[TARGET_GOAL_CNT] = mem4[GOAL_CNT];
		   mem4[WARNING_HIGH] = mem4[GOAL_CNT];
		   mem4[TARGET_CMP] = mem4[GOAL_CNT];
		}
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
				if(mem4[TARGET_COUNT_SENSOR]) //LOCAL SENSOR 
				{
					Alarm_Start();
				}
				else //PLC
				{
					Alarm_Stop();
					mem4[COUNT] = count_number;
				}
				mem4[PRESSURE] = current_pressure;
				if((mem4[GOAL_CNT] == mem4[COUNT])&& mem4[MACHINE_STATES] == NORMAL)
				{
					mem4[MACHINE_STATES] = COMPLETE_PRODUCTS;
				}
			}
			else
			{
				cbi(PORTB,7);
				GetExceptionCode(&exception,0x01,0x06);
				sb->SerialWrite((char*)&exception,sizeof(exception)); //리스폰스 데이터 쓰기.
				cbi(UCSR1B,RXCIE0);
				sb->SerialFlush();
				sbi(UCSR1B,RXCIE0);
				sbi(PORTB,7);
				
			}
		}
		
	}
}

#if USE_ETH
static void proc2(void* pvParam)
{
RESET_ETH:
    char led_flag = 0;
	static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x24};
	static uint8_t myip[4] = {0,0,0,0};
	memcpy(myip,cmp_mem,sizeof(cmp_mem));		
	static uint8_t buf[BUFFER_SIZE+1];
	uint16_t plen;
	DDRB = 0xff;
	PORTB = 0xff;
	 enc28j60Init(mymac);
	 enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
	 vTaskDelay(10);
	 enc28j60PhyWrite(PHLCON,0x476);
	 vTaskDelay(20);
	 //init the ethernet/ip layer:
	 init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);

	 while(1)
	 {
		 if((cmp_mem[0] != mem4[IPV4_0]) | (cmp_mem[1] != mem4[IPV4_1]) | (cmp_mem[2] != mem4[IPV4_2]) | (cmp_mem[3] != mem4[IPV4_3]))
		 {
			 if(cmp_mem[0] != mem4[IPV4_0])
			 {
				  eeprom_update_byte((uint8_t*)0,mem4[IPV4_0]);
				  cmp_mem[0] = mem4[IPV4_0];
			 }
			 if(cmp_mem[1] != mem4[IPV4_1])
			 {
				  eeprom_update_byte((uint8_t*)1,mem4[IPV4_1]);
				  cmp_mem[1] = mem4[IPV4_1];
			 }
			 if(cmp_mem[2] != mem4[IPV4_2])
			 {
				  eeprom_update_byte((uint8_t*)2,mem4[IPV4_2]);
				  cmp_mem[2] = mem4[IPV4_2];
			 }
			 if(cmp_mem[3] != mem4[IPV4_3])
			 {
				  eeprom_update_byte((uint8_t*)3,mem4[IPV4_3]);
				  cmp_mem[3] = mem4[IPV4_3];
			 }
			 goto RESET_ETH;
		 }
		 plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
		 if(plen==0)
		 {
			 goto UDP_SEND;
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
		 if(buf[UDP_DATA_P] == 0x01 + '0')
		 {
			 char temp[29] = {0};
			 char loop = buf[UDP_DATA_P + 1];
			 loop = loop - '0';
			 char start = 2;
			 char num = 0;
			 for(char i=1;i<loop+1;i++)
			 {
				 temp[i-1] = buf[UDP_DATA_P + (start + (i-1))];
				 if(i % 2 == 0)
				 {
					mem4[UDP_DATA0 + num] =  (( 0xff00 & temp[i-1] << 8)) | (0x00ff & temp[i-2]);
					num++;
				 }
			 }
			 mem4[GOAL_CNT] = mem4[UDP_DATA14];
			 mem4[TARGET_MAX] = mem4[GOAL_CNT];
			 mem4[TARGET_GOAL_CNT] = mem4[GOAL_CNT];
			 mem4[WARNING_HIGH] = mem4[GOAL_CNT];
			 mem4[TARGET_CMP] = mem4[GOAL_CNT];
			 goto UDP_SEND;
		 }
		 UDP_SEND:
			 led_flag = ~led_flag;
			 if(led_flag)
			 {
				 PORTB = sbi(PORTB,4);
			 }
			 else
			 {
				 PORTB = cbi(PORTB,4); 
			 }
			 static int data[8] = {0};
			 data[0] = mem4[TEMP];
			 data[1] = mem4[COUNT];
			 data[2] = mem4[PRESSURE];
			 data[3] = mem4[GOAL_CNT];
			 data[4] = mem4[MACHINE_STATES];
			 data[5] = mem4[SEC];
			 data[6] = mem4[MIN];
			 data[7] = mem4[HOUR];
			 make_udp_reply_from_request(buf,(char*)&data,sizeof(data),MYUDPPORT);
			 vTaskDelay(100);
	}
	  
}
#endif


void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

}





