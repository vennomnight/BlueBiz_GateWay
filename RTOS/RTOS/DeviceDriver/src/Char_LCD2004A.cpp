/*
 * Char_LCD2004A.cpp
 *
 * Created: 2017-09-12 오전 10:36:05
 *  Author: bluebiz
 */ 

#include "Char_LCD2004A.h"

void Char_LCD2004A::Char_LCD2004A::Device_Init()
{
	LCD_DATA_PORT = 0xff;
	this->Command_Set(0x28);
	this->Command_Set(0x06);
	this->Command_Set(0x0c);
	this->Clear_Lcd();
}
Char_LCD2004A::Char_LCD2004A()
{
	DDRA = 0xff;
	this->Device_Init();
}
inline void Char_LCD2004A::LCD_E_HIGH(void)
{
	LCD_CTRL_PORT |= 0x02;
}
inline void Char_LCD2004A::LCD_E_LOW(void)
{
	LCD_CTRL_PORT &= 0xfd;
}
inline void Char_LCD2004A::LCD_RS_HIGH(void)
{
	LCD_CTRL_PORT |= 0x01;
}
inline void Char_LCD2004A::LCD_RS_LOW(void)
{
	LCD_CTRL_PORT &= 0xfe;
}
inline void Char_LCD2004A::E_pulse()
{
	this->LCD_E_HIGH();
	this->delay(1);
	this->LCD_E_LOW();
}
void Char_LCD2004A::delay(char delay_ms)
{
	char i;
	for(i=0;i<delay_ms;i++)
	{
		_delay_ms(0.5);
	}
}
void Char_LCD2004A::Command_Set(char cmd)
{
	this->LCD_RS_LOW();
	LCD_DATA_PORT = (cmd & 0xf0);
	this->E_pulse();
	LCD_DATA_PORT = (cmd << 4 );
	this->E_pulse();
}
void Char_LCD2004A::Data_set(char data)
{
	LCD_RS_HIGH();
	LCD_DATA_PORT = (data & 0xf0) | 0x01;
	E_pulse();
	LCD_DATA_PORT = (data << 4) | 0x01;
	E_pulse();
}
void Char_LCD2004A::Clear_Lcd(void)
{
	this->Command_Set(0x01);
	delay(5);
}
void Char_LCD2004A::Cursor_Set(char x,char y)
{
	switch (y)
	{
		case 0: y = 0x80; break;
		case 1: y = 0xc0; break;
		case 2: y = 0x94; break;
		case 3: y = 0xd4; break;
	}
	y = y + x;
	this->Command_Set(y);
}
void Char_LCD2004A::Set_Cursor_Print(char x,char y,const char* str)
{
	this->Cursor_Set(x,y);
	while(*str)
	{
		this->Data_set(*str++);
	}
}
void Char_LCD2004A::Cursor_Home(void)
{
	this->Command_Set(0x02);
	this->delay(5);
}
void Char_LCD2004A::Move_Display(char point)
{
	if(point == LEFT) this->Command_Set(0x18);
	else if (point == RIGHT) this->Command_Set(0x1c);
}
void Char_LCD2004A::Move_Cursor(char point)
{
	if(point == RIGHT) this->Command_Set(0x14);
	else if (point == LEFT) this->Command_Set(0x10);
}
void Char_LCD2004A::Entry_Shift(char point)
{
	if(point == RIGHT) this->Command_Set(0x05);
	else if(point == LEFT) this->Command_Set(0x07);
	else if(point == NO) this->Command_Set(0x06);
}
void Char_LCD2004A::Display_On_Off(uint8_t d,uint8_t c,uint8_t b)
{
		unsigned char display = 0x08;
		if(d == ON) d = 0x04;
		else d = 0x00;
		if(c == ON) c = 0x02;
		else c = 0x00;
		if(b == ON) b = 0x01;
		else b =0x00;
		
		display = display | d | c | b;
		this->Command_Set(display);
}
void Char_LCD2004A::Register_Font(char addr,const uint8_t* font)
{
	if(addr == 0)
	this->Command_Set(0x40);
	else
	{
		addr *= 8;
		this->Command_Set(0x40 + addr);
	}
	if(addr > 64)
	{
		return;
	}
	for(uint8_t i = 0; i < 8; i++)
	{
		this->Data_set(font[i]);
	}
}
void Char_LCD2004A::Device_Writes(const char* data)
{
		while(*data)
		{
			this->Data_set(*data++);
		}
}
void Char_LCD2004A::Font_Print(char addr)
{
	if(addr < 8)
		this->Data_set(addr);
}
void Char_LCD2004A::operator delete(void* ptr)
{
	free(ptr);
}
void* Char_LCD2004A::operator new(size_t size)
{
	return malloc(size);
}


