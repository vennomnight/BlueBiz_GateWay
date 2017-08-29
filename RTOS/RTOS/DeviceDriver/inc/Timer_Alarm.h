/*
 * Timer_Alarm.h
 *
 * Created: 2017-08-24 오후 7:14:13
 *  Author: bluebiz
 */ 


#ifndef TIMER_ALARM_H_
#define TIMER_ALARM_H_
extern int mem4[];
class Timer_Alarm
{
private:
	static int *ptr;
public:
	explicit Timer_Alarm();
	static void Service_routine() __asm__("__vector_9") __attribute__((__signal__, __used__, __externally_visible__));
};	
	
#endif /* TIMER_ALARM_H_ */