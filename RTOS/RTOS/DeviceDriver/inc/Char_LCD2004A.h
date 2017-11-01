/*
 * Char_LCD2004A.h
 *
 * Created: 2017-09-12 오전 10:31:31
 *  Author: bluebiz
 */ 


#ifndef CHAR_LCD2004A_H_
#define CHAR_LCD2004A_H_

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#include "FreeRTOS.h"
#include "semphr.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

#include "DeviceDriverInterface.h"
#define LCD_DATA_DIR  DDRA
#define LCD_DATA_PORT PORTA
#define LCD_CTRL_DIR  DDRA
#define LCD_CTRL_PORT PORTA
#define RIGHT 1
#define LEFT 2
#define ON 1
#define OFF 2
#define NO 0

class Char_LCD2004A : public DeviceDriveInterFace
{
private:
	inline void LCD_E_HIGH(void);
	inline void LCD_E_LOW(void);
	inline void LCD_RS_HIGH(void);
	inline void LCD_RS_LOW(void);
	inline void E_pulse(void);
	void delay(char delay_ms);
	void Command_Set(char cmd);
	void Data_set(char data);
	virtual void Device_Init();
	
public:
	explicit Char_LCD2004A();

	void Clear_Lcd(void);
	void Cursor_Set(char x,char y);
	void Set_Cursor_Print(char x,char y,const char* str);
	void Cursor_Home(void);
	void Move_Display(char point);
	void Move_Cursor(char point);
	void Entry_Shift(char point);
	void Display_On_Off(uint8_t d,uint8_t c,uint8_t b);
	void Register_Font(char addr,const uint8_t* font);
	void Device_Writes(const char* data);
	void Font_Print(char addr);
	void Lcd_Print(const char* str);
	void* operator new(size_t size);
	void operator delete(void* ptr);
	
};

#endif /* CHAR_LCD2004A_H_ */