/*
 * RTOS.cpp
 *
 * Created: 2016-11-26 오후 2:50:59
 * Author : kimkisu
 */ 


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

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
#include "Char_LCD2004A.h"
#include "Adc.h"
#include "DFRobotHighTemperatureSensor.h"
//////////////////


const char debug = 0; //처음 부팅 디버깅용 
const uint16_t device_serial=0xF001; //이 디바이스 시리얼번호
#define USE_ETH 1 //이더넷 사용
#define USE_HMI 1 //HMI 사용

#define PACKET_DEBUG 0 // PLC 0 으로 하고 해야함,, 
#define USE_PLC 1 //PLC 사용 
#define USE_ADC 1 //ADC핀 사용 // 외부 온도 센서 사용시 

#define CHECK_ERROR 0 //에러 디버깅용
#define USE_SYSTEM_SEC 1 // 시스템 초 사용
#define ADC_LOOP 30
//#define EXTERNAL_COUNT_SENSOR 1 // 외부 센서 사용시


//#define sbi(PORTX, BitX) PORTX |= (1 <<BitX)
//#define cbi(PORTX, BitX) PORTX &= ~(1 << BitX)


#if USE_ETH
#define	BUFFER_SIZE 100
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
	START_BUTTON,
	STOP_BUTTON,
	CURRENT_PAGE,
	CURRENT_STATE_ON,
	CURRENT_RUN_NUMBER,
	DATE_DATA0,
	DATE_DATA1,
	DATE_DATA2,
	DATE_DATA3,
	DATE_DATA4,
	DATE_DATA5,
	DATE_DATA6,
	DATE_DATA7,
	DATE_DATA8,
	DATE_DATA9,
	DATE_DATA10,
	DATE_DATA11,
	DATE_DATA12,
	DATE_DATA13,
	LOCAL_TEMP_SENSOR,
	LOCAL_PT100_SENSOR,
	LOCAL_NTC_SENSOR,
	//#if USE_SYSTEM_SEC
		//SYSTEM_SEC_CLOCK,
	//#endif
	MAX_ENUM,		
};
enum
{
	PAGE_1_CURRENT_CNT,
	PAGE_2_CURRENT_CNT,
	PAGE_3_CURRENT_CNT,
	PAGE_4_CURRENT_CNT,
	PAGE_5_CURRENT_CNT,
	PAGE_6_CURRENT_CNT,
	PAGE_7_CURRENT_CNT,
	PAGE_8_CURRENT_CNT,		
	PAGE_9_CURRENT_CNT,
	PAGE_10_CURRENT_CNT,
	PAGE_MAX
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
static void proc3(void* pvParam);
static void proc4(void* pvParam);


/*
 * Idle hook is used to scheduler co-routines.
 */
extern "C"
{
	void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );
	void vApplicationIdleHook( void );

}
Dev_Manager *dev;
Char_LCD2004A *lcd;
#if USE_ADC
	Adc * adc;
#endif

void *DataStruct[MAX] = {nullptr}; //Dev_Manager Enum Using
void Init_Dev();
void Uart_ISR(Dev_type Device,uint16_t Arg);
void RS485_ISR(Dev_type Device,uint16_t Arg);
void Timer_ISR(Dev_type Device,uint16_t Arg);
void Count_Sensor_ISR(Dev_type Device,uint16_t Arg);
void ADC_ISR(Dev_type Device,uint16_t Arg);

void Set_Alarm();

static void System_Init();
float fnCalTemp(float lfOneVolt);
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
//현재 카운터를 동작하는지 알 수 있는 상태 변수 
uint8_t current_states_lookup_table[PAGE_MAX]={0};
uint32_t current_states_times;

int mem4[MAX_ENUM] = {0};  //function 10 메모리 공간
int current_cnt_mem[PAGE_MAX] = {0}; //각 페이지별 담는 현재 카운터 수 	
char cmp_mem[4] = {0};  //IP 주소 비교 메모리 
uint8_t Ctl_LCD_Cursor = 0 ; // LCD커서 
uint8_t chatter_flag = 0; //채터링 방지 플래그 변수
volatile uint8_t lcd_cnt = 0;//lcd 제어용 변수 
volatile uint8_t cls_var = 0;
static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x24};
static uint8_t myip[4] = {0,0,0,0};


//uint8_t use_external_count_sensor = 0; // 0 PLC  1 EXTERNAL COUTN SENSOR
#if USE_ADC
	uint16_t Adc_channels[8] = {0};
	uint8_t Adc_check_flag[8] = {0};
#endif
int main( void )
{
	mem4[CURRENT_PAGE] = 0;//초기 페이지는 1 
	System_Init();
	cli();  //인터럽트 금지 
	Init_Dev(); //dev 매니저 초기화
	
	
	dev->Open_Handle(UART0,Uart_ISR);  //드라이버 매니져에 인터럽트 루틴 등록
	dev->Open_Handle(RS485,RS485_ISR); //드라이버 매니져에 인터럽트 루틴 등록
	dev->Open_Handle(SEC_TIMER,Timer_ISR);
	dev->Open_Handle(COUNT_SENSOR,Count_Sensor_ISR);
	#if USE_ADC
		dev->Open_Handle(_ADC,ADC_ISR);
	#endif
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
	200,                   //스택의 크기
	sb,       // 테스크 매개 변수
	2,                     //테스크 우선 순위
	NULL                   //태스크 핸들
	);
	
		xTaskCreate(proc1,                //테스크 실행할 함수 포인터
		"Task2",      //테스크 이름
		150,                   //스택의 크기
		sb1,       // 테스크 매개 변수
		2,                     //테스크 우선 순위
		NULL                   //태스크 핸들
		);
		
		#if USE_ETH
				xTaskCreate(proc2,                //테스크 실행할 함수 포인터
				"Task3",      //테스크 이름
				600,                   //스택의 크기
				NULL,       // 테스크 매개 변수
				2,                     //테스크 우선 순위0.
				NULL                   //태스크 핸들
				);
		#endif
		xTaskCreate(proc4,                //테스크 실행할 함수 포인터
				"Task4",      //테스크 이름
				300,                   //스택의 크기
				NULL,       // 테스크 매개 변수
				2,                     //테스크 우선 순위0.
				NULL                   //태스크 핸들
		);
		
		#if USE_ADC
		xTaskCreate(proc3,                //테스크 실행할 함수 포인터
		"Task3",      //테스크 이름
		80,                   //스택의 크기
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
	lcd = new Char_LCD2004A();
	#if USE_ADC
		adc = new Adc();
		dev->Register_Dev(adc,_ADC);
	#endif
	dev->Register_Dev(new UartDriver,UART0);
	dev->Register_Dev(new RS485Driver,RS485);
	dev->Register_Dev(new Timer,SEC_TIMER);
	dev->Register_Dev(new Count_Sensor,COUNT_SENSOR);
	
	dev->Device_Init(UART0);
	dev->Device_Init(RS485);
	dev->Device_Init(SEC_TIMER);
	dev->Device_Init(COUNT_SENSOR);
	#if USE_ADC
		dev->Device_Init(_ADC);
	#endif
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
	Ctl_LCD_Cursor++;
	cbi(PORTB,5); //통신 램프 클리어
	cbi(PORTB,6); //통신 램프 클리어 	
	#if USE_SYSTEM_SEC
		//mem4[SYSTEM_SEC_CLOCK]++;
		current_states_times++;
	#endif
}
void ADC_ISR(Dev_type Device,uint16_t Arg)
{

}
void Count_Sensor_ISR(Dev_type Device,uint16_t Arg)
{
	Alarm_Open(ALARM0,80,Set_Alarm);
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
	#if PACKET_DEBUG
		SerialBuffer *sb1 = (SerialBuffer*)DataStruct[RS485];
	#endif
	SerialBuffer *sb = static_cast<SerialBuffer*>(pvParam);	
	register uint16_t i;
	while(1)
	{
		//mem4[COUNT] = PIND;
		if(PIND == 0xfe && chatter_flag == 1)//
		{
			if(mem4[START_BUTTON] == ON)
			{
				if(current_states_lookup_table[mem4[CURRENT_RUN_NUMBER]] == ON)
				{
					current_cnt_mem[mem4[CURRENT_RUN_NUMBER]]++;
				}
			}
			chatter_flag = 2;
		}
		if(PIND == 0xff && chatter_flag == 2)//
		{
			vTaskDelay(10);
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
		mem4[COUNT] = current_cnt_mem[mem4[CURRENT_PAGE]];
		if(read_Flag == 0)
		{
			if(sb->SerialAvailable() >= 2)
			{
				for(i=0;i<2;i++)
				{
					buf1[i] = sb->SerialRead();
					#if PACKET_DEBUG
						sb1->SerialWrite(buf1[i]);
					#endif
				}
				if(buf1[0] != 0x01)
				{
				//	read_Flag = 0;
					goto FREAM_ERROR;
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
					FREAM_ERROR:
					#if CHECK_ERROR
						mem4[ERROR_CNT]++;
					#endif
					GetExceptionCode(&exception,0x01,0x01);  
					cbi(PORTB,7);
					cbi(UCSR0B,RXCIE0);
					dev->getInterfaceAddr(UART0)->Stop_Device();                                                                                                                                                                                                                
					sb->SerialFlush();
					dev->getInterfaceAddr(UART0)->Start_Device(0);
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
						#if PACKET_DEBUG
							sb1->SerialWrite(buf1[i]);
						#endif
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
						#if PACKET_DEBUG
							sb1->SerialWrite(buf1[i]);
						#endif
					}
					int tempadr =((0xff & buf1[2] << 8) | 0xff & buf1[3]);
					if(tempadr >= MAX_ENUM)  //주소 사이즈 체크 
					{
							//GetExceptionCode(&exception,0x01,0x02);
							//sb->SerialWrite((char*)&exception,sizeof(exception));
							goto FREAM_ERROR;
					}
					else
					{
							GetFunc04Data(buf1,&func04,mem4);
							sb->SerialWrite((char*)&func04,sizeof(func04));
							read_Flag = 0;
					}

				}
			}
			else if(function_code == 0x05) //터치 버튼 
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(i=2;i<8;i++)
					{
						buf1[i] = sb->SerialRead();
						#if PACKET_DEBUG
						sb1->SerialWrite(buf1[i]);
						#endif
					}
					int tempadr =((0xff & buf1[2] << 8) | 0xff & buf1[3]);
					if(tempadr >= MAX_ENUM)  //주소 사이즈 체크
					{
						goto FREAM_ERROR;
					}
					else
					{
						GetFunc05Data(buf1,&func05);
						sb->SerialWrite((char*)&func05,sizeof(func05));
						read_Flag = 0;
					}
	
				}
			}
			else if(function_code == 0x10)  //3_MAX1W 입력 
			{
				if(sb->SerialAvailable() >= 9)
				{
					for(i=2;i<11;i++)
					{
						buf1[i] = sb->SerialRead();
						#if PACKET_DEBUG
						sb1->SerialWrite(buf1[i]);
						#endif
					}
					GetFucc10Data(buf1,&func10,mem4); //데이터 파싱
					int tempadr =((0xff & buf1[2] << 8) | 0xff & buf1[3]);
					if(tempadr >= MAX_ENUM)  //주소 사이즈 체크
					{
						goto FREAM_ERROR;
					}
					else
					{
						if(func10.startingAddressLo == 38)
						{
							if(current_states_lookup_table[mem4[CURRENT_PAGE]] == 0) //룩업 테이블은 처음 시작시 0값이어야 함,
							{
								uint8_t flag = 0;
								mem4[START_BUTTON] = ON;
								mem4[CURRENT_STATE_ON] = ON; // 시작 등 ON
								//룩업 테이블을 먼저 확인 함. 한개라도 가동중이면 리턴함, 가동이 반드시 끝나고 시작해야함(페이지 이동 후 시작 시)
								for(uint8_t i = 0; i< PAGE_MAX;i++)
								{
									if(current_states_lookup_table[i] == ON)
									{
										flag = 1;
										break;
									}
								}
								if(flag == 0)
								{
									current_states_lookup_table[mem4[CURRENT_PAGE]] = ON;
									mem4[CURRENT_RUN_NUMBER] = mem4[CURRENT_PAGE];
								}
							}
						}
						if(func10.startingAddressLo == 39)
						{
						
							if(current_states_lookup_table[mem4[CURRENT_PAGE]] == ON)
							{
								current_states_lookup_table[mem4[CURRENT_PAGE]] = 0;
								mem4[START_BUTTON] = OFF;
								mem4[CURRENT_STATE_ON] = 0;//시작 등  OFF
							}
						}
						ResponseFucc10Data(buf1,&rsp10); //리스폰스 데이터를 만듬.
						sb->SerialWrite((char*)&rsp10,sizeof(rsp10)); //리스폰스 데이터 쓰기.
						read_Flag = 0;
					}
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
					if(mem4[START_BUTTON] == ON)
					{
						uint8_t i;
						for(i=0;i<PAGE_MAX;i++)
						{
							if(current_states_lookup_table[i] == ON)
							{
								current_cnt_mem[i] = count_number;
								break;
							}
						}
						mem4[COUNT] = current_cnt_mem[i]; 
					}

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
	//lcd->Clear_Lcd();
	//static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x24};
	//static uint8_t myip[4] = {0,0,0,0};
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
			 cls_var = 1;
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
		 if(buf[UDP_DATA_P] == 0x01)  //리눅스 서버용 파싱 
		 {
			 char temp[16] = {0};
			 char loop = buf[UDP_DATA_P + 1];
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
			  mem4[GOAL_CNT] =  (0xff00 & (buf[UDP_DATA_P + 14] << 8))| (0x00ff & buf[UDP_DATA_P + 15]);
			  mem4[TARGET_MAX] = mem4[GOAL_CNT];
			  mem4[TARGET_GOAL_CNT] = mem4[GOAL_CNT];
			  mem4[WARNING_HIGH] = mem4[GOAL_CNT];
			  mem4[TARGET_CMP] = mem4[GOAL_CNT];
			  goto UDP_SEND;
			 
		 }
		 if(buf[UDP_DATA_P] == 0x01 + '0') // 기존 라이브러리 사용 (자바용) 프로덕트 이름 시리얼번호 카운트 받음
		 {
			 char temp[29] = {0};
			 char loop = buf[UDP_DATA_P + 1];
			 loop = loop - '0';
			 if(loop <= 1 | loop > 30)
			 {
				 goto UDP_SEND;
			 }
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
		 else if(buf[UDP_DATA_P] == 0x02 + '0')  //start time from Server
		 {
			 char temp[14] = {0};
			 char loop = buf[UDP_DATA_P + 1];
			 loop = loop - '0';
			 if(loop <= 1 | loop > 14)
			 {
				goto UDP_SEND;
			 }
			 char start = 2;
			 char num = 0;
			 for(char i=1;i<loop+1;i++)
			 {
				  temp[i-1] = buf[UDP_DATA_P + (start + (i-1))];
				  if(i % 2 == 0)
				  {
					  mem4[DATE_DATA0 + num] =  (( 0xff00 & temp[i-1] << 8)) | (0x00ff & temp[i-2]);
					  num++;
				  }
			 }
			 
		 }
		 else if(buf[UDP_DATA_P] == 0x03 + '0') //end time fromServer
		 {
			 char temp[14] = {0};
			 char loop = buf[UDP_DATA_P + 1];
			 loop = loop - '0';
			 if(loop <= 1 | loop > 14)
			 {
				 goto UDP_SEND;
			 }
			 char start = 2;
			 char num = 0;
			 for(char i=1;i<loop+1;i++)
			 {
				 temp[i-1] = buf[UDP_DATA_P + (start + (i-1))];
				 if(i % 2 == 0)
				 {
					 mem4[DATE_DATA7 + num] =  (( 0xff00 & temp[i-1] << 8)) | (0x00ff & temp[i-2]);
					 num++;
				 }
			 }
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
			 static int data[19] = {0};
			 data[0] = mem4[TEMP];
			 data[1] = mem4[COUNT];
			 data[2] = mem4[PRESSURE];
			 data[3] = mem4[GOAL_CNT];
			 data[4] = mem4[MACHINE_STATES];
			 data[5] = mem4[SEC];
			 data[6] = mem4[MIN];
			 data[7] = mem4[HOUR];
			 data[8] = mem4[CURRENT_RUN_NUMBER];  //현재 생산 페이지
			 data[9] = mem4[CURRENT_STATE_ON]; //혀재 생산 상태 값을 보냄. 가동중 or 가동 아닌 상태
			 data[10] = mem4[START_BUTTON]; //현재 시작 버튼의 상태를 보냄
			 data[11] = mem4[CURRENT_PAGE]; //현재 뷰 페이지 정보 
			 data[12] = device_serial;
			 data[13] = mem4[UDP_DATA0];
			 data[14] = mem4[UDP_DATA1];
			 data[15] = mem4[UDP_DATA2];
			 data[16] = mem4[UDP_DATA3];
			 data[17] = mem4[UDP_DATA4];
			 data[18] = mem4[UDP_DATA5];
			 make_udp_reply_from_request(buf,(char*)&data,sizeof(data),MYUDPPORT);
			// memcpy(buf,data,sizeof(data));
			 //enc28j60PacketSend(UDP_HEADER_LEN+IP_HEADER_LEN+ETH_HEADER_LEN+sizeof(data),buf);
			 //vTaskDelay(500);
			 //memset(buf,0,sizeof(buf));
			 //vTaskDelay(100);
			 lcd_cnt++;
			taskYIELD();
			 
	}
	  
}
#endif

const float g_alfVolt[161] PROGMEM = {
	0.26859, 0.28237, 0.29671, 0.31165, 0.32719, 0.34335, 0.36015, 0.37760, 0.39571, 0.41449,
	0.43397, 0.45415, 0.47505, 0.49667, 0.51902, 0.54213, 0.56598, 0.59060, 0.61599, 0.64215,
	0.66909, 0.69681, 0.72532, 0.75462, 0.78470, 0.81557, 0.84721, 0.87964, 0.91283, 0.94678,
	0.98149, 1.01694, 1.05312, 1.09002, 1.12762, 1.16590, 1.20484, 1.24444, 1.28465, 1.32547,
	1.36687, 1.40881, 1.45129, 1.49426, 1.53769, 1.58157, 1.62585, 1.67051, 1.71551, 1.76082,
	1.80641, 1.85224, 1.89827, 1.94448, 1.99083, 2.03728, 2.08380, 2.13035, 2.17691, 2.22343,
	2.26988, 2.31623, 2.36245, 2.40850, 2.45436, 2.50000, 2.54538, 2.59049, 2.63528, 2.67974,
	2.72385, 2.76757, 2.81089, 2.85379, 2.89624, 2.93823, 2.97973, 3.02074, 3.06125, 3.10122,
	3.14066, 3.17955, 3.21788, 3.25565, 3.29284, 3.32944, 3.36546, 3.40089, 3.43572, 3.46995,
	3.50358, 3.53661, 3.56903, 3.60085, 3.63208, 3.66270, 3.69273, 3.72216, 3.75101, 3.77927,
	3.80695, 3.83406, 3.86060, 3.88658, 3.91199, 3.93687, 3.96119, 3.98499, 4.00825, 4.03100,
	4.05323, 4.07497, 4.09620, 4.11695, 4.13722, 4.15702, 4.17637, 4.19525, 4.21370, 4.23171,
	4.24929, 4.26645, 4.28321, 4.29956, 4.31553, 4.33110, 4.34631, 4.36114, 4.37562, 4.38974,
	4.40352, 4.41697, 4.43009, 4.44289, 4.45537, 4.46755, 4.47944, 4.49103, 4.50234, 4.51337,
	4.52413, 4.53463, 4.54487, 4.55486, 4.56461, 4.57411, 4.58339, 4.59244, 4.60126, 4.60987,
	4.61828, 4.62647, 4.63447, 4.64227, 4.64989, 4.65732, 4.66457, 4.67164, 4.67855, 4.68528,
4.69186 };
const int g_adTemp[161] PROGMEM = {
	-40, -39, -38, -37, -36, -35, -34, -33, -32, -31,
	-30, -29, -28, -27, -26, -25, -24, -23, -22, -21,
	-20, -19, -18, -17, -16, -15, -14, -13, -12, -11,
	-10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
	60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
	70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
	80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
	90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
	100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
	110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
120};

float fnCalTemp(float lfOneVolt)
{
	uint8_t i = 0;
	uint8_t dClass = 0;
	float lfCalTemp = 0;
	
	for( i=0; i<161; i++ )
	{
		if( lfOneVolt<pgm_read_float(&g_alfVolt[i]) )
		{
			dClass = i;
			break;
		}
	}
	if (pgm_read_dword(&g_adTemp[dClass-1])<0 )
	{
		//정답지: 32~40 간략화           //1도   상위 고정값 - 측정된 값    /1도 구간에서 저항 변환량
		lfCalTemp = pgm_read_dword(&g_adTemp[dClass-1]) + (1 * ( (pgm_read_float(&g_alfVolt[dClass])-lfOneVolt)/(pgm_read_float(&g_alfVolt[dClass])-pgm_read_float(&g_alfVolt[dClass-1])) ));
	}
	else
	{
		lfCalTemp = pgm_read_dword(&g_adTemp[dClass]) - (1 * ( (pgm_read_float(&g_alfVolt[dClass])-lfOneVolt)/(pgm_read_float(&g_alfVolt[dClass])-pgm_read_float(&g_alfVolt[dClass-1])) ));
	}
	return lfCalTemp;
}
static void proc3(void* pvParam)
{
	DFRobotHighTemperature PT100(5.000);
	uint8_t cnt = 0;
	uint8_t seq = 0;
	uint8_t cnt1 = 0;
	uint8_t cnt2 = 0;
	while(1)
	{
		adc->Start_Device(seq % 3);
		uint8_t channel = ADMUX & 0x1f; 
		vTaskDelay(10);
		if(channel == 0)
		{
			uint16_t read = ADCL+((uint16_t)ADCH << 8);
			float temp = read * 4.8828125;
			Adc_channels[channel] = Adc_channels[channel] + (temp / 10);
			cnt++;
		}
		else if(channel == 1)
		{
			uint16_t read = ADCL+((uint16_t)ADCH << 8);
			Adc_channels[channel] +=PT100.readTemperature(read);
			//mem4[LOCAL_PT100_SENSOR] = read;
			cnt1++;
		}
		else if(channel == 2)
		{
			uint16_t read = ADCL+((uint16_t)ADCH << 8);
			Adc_channels[channel] += read;
			cnt2++;
		}
		
		if(cnt >= ADC_LOOP)
		{
			mem4[LOCAL_TEMP_SENSOR] = Adc_channels[0] / ADC_LOOP;
			Adc_channels[0] = 0;
			cnt = 0;
		}
		else if(cnt1 >= ADC_LOOP)
		{
			unsigned int temp = Adc_channels[1] / ADC_LOOP;
			//int temp1 = PT100.readTemperature(temp);
			mem4[LOCAL_PT100_SENSOR] = temp - 5;
			Adc_channels[1] = 0;
			cnt1 = 0;
		}
		else if(cnt2 >= ADC_LOOP)
		{
			float vin = (Adc_channels[2] / ADC_LOOP )* 0.004887;
			float temp = fnCalTemp(vin);
			mem4[LOCAL_NTC_SENSOR] = temp * 10;
			Adc_channels[2] = 0;
			
			cnt2 = 0;
		}
		seq++;
	}
	
}
static void proc4(void* pvParam)
{
	char ip_adr1[12];
		 lcd->Clear_Lcd();
		 unsigned char left_up[8]  = {0x01,0x02,0x04,0x08,0x10,0x10,0x10,0x10};
		 unsigned char right_up[8] = {0x10,0x08,0x04,0x02,0x01,0x01,0x01,0x01};
		 unsigned char right_down[8] = {0x01,0x01,0x01,0x01,0x02,0x04,0x08,0x10};
		 unsigned char left_down[8] = {0x10,0x10,0x10,0x10,0x08,0x04,0x02,0x01};
		 
		 
		 unsigned char left_pic[8] = {0x01,0x02,0x06,0x0E,0x1E,0x16,0x12,0x11};
		 unsigned char right_pic[8] = {0x10,0x08,0x0C,0x0E,0x0F,0X0D,0x09,0x11};
		 unsigned char right_down_pic[8] = {0x11,0x09,0x0D,0x0F,0x0E,0x0C,0x08,0x10};
		 unsigned char left_down_pic[8] = {0x11,0x12,0x1E,0x1E,0x0E,0x06,0x02,0x01};
		 
		 
		 
		 lcd->Register_Font(0,left_up);
		 lcd->Register_Font(1,right_up);
		 lcd->Register_Font(2,right_down);
		 lcd->Register_Font(3,left_down);
		 
		 lcd->Register_Font(4,left_pic);
		 lcd->Register_Font(5,right_pic);
		 lcd->Register_Font(6,right_down_pic);
		 lcd->Register_Font(7,left_down_pic);
		 LCD_CLEAR:
			lcd->Clear_Lcd();
	while(1)
	{
		if(cls_var == 1)
		{
			cls_var = 0;
			goto LCD_CLEAR;
		}
		lcd->Cursor_Home();
		lcd->Device_Writes("IP :");
		sprintf(ip_adr1,"%d",myip[0]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(".");
		sprintf(ip_adr1,"%d",myip[1]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(".");
		sprintf(ip_adr1,"%d",myip[2]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(".");
		sprintf(ip_adr1,"%d",myip[3]);
		lcd->Device_Writes(ip_adr1);
		lcd->Set_Cursor_Print(0,1,"MAC:");
		sprintf(ip_adr1,"%x",mymac[0]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(":");
		sprintf(ip_adr1,"%x",mymac[1]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(":");
		sprintf(ip_adr1,"%x",mymac[2]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(":");
		sprintf(ip_adr1,"%x",mymac[3]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(":");
		sprintf(ip_adr1,"%x",mymac[4]);
		lcd->Device_Writes(ip_adr1);
		lcd->Device_Writes(":");
		sprintf(ip_adr1,"%x",mymac[5]);
		lcd->Device_Writes(ip_adr1);
					 //char num = Ctl_LCD_Cursor % 4;
		char cnts1 = lcd_cnt / 30;
		char num = cnts1 % 4;
		if(num == 0)
		{
			lcd->Cursor_Home();
			lcd->Cursor_Set(0,2);
			lcd->Font_Print(0x04);
			lcd->Font_Print(0x01);
			lcd->Lcd_Print("Sensor Gateway");
			lcd->Cursor_Set(0,3);
			lcd->Font_Print(0x03);
			lcd->Font_Print(0x02);
			lcd->Lcd_Print("BLUE-BIZ.CO.LTD");
		}
		else if(num == 1)
		{
			 lcd->Cursor_Home();
			 lcd->Cursor_Set(0,2);
			 lcd->Font_Print(0x00);
			 lcd->Font_Print(0x05);
			 lcd->Lcd_Print("Sensor Gateway");
			 lcd->Cursor_Set(0,3);
			 lcd->Font_Print(0x03);
			 lcd->Font_Print(0x02);
			 lcd->Lcd_Print("BLUE-BIZ.CO.LTD");
		}
		else if(num == 2)
		{
			lcd->Cursor_Home();
			lcd->Cursor_Set(0,2);
			lcd->Font_Print(0x00);
			lcd->Font_Print(0x01);
			lcd->Lcd_Print("Sensor Gateway");
			lcd->Cursor_Set(0,3);
			lcd->Font_Print(0x03);
			lcd->Font_Print(0x06);
			lcd->Lcd_Print("BLUE-BIZ.CO.LTD");
		}
		else if(num == 3)
		{
			 lcd->Cursor_Home();
			 lcd->Cursor_Set(0,2);
			 lcd->Font_Print(0x00);
			 lcd->Font_Print(0x01);
			 lcd->Lcd_Print("Sensor Gateway");
			 lcd->Cursor_Set(0,3);
			 lcd->Font_Print(0x07);
			 lcd->Font_Print(0x02);
			 lcd->Lcd_Print("BLUE-BIZ.CO.LTD");
		}
	}
	
}
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

}





