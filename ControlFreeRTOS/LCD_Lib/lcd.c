/*
 * lcd.c
 *
 *  Created on: 23.05.2012
 *      Author: lamazavr
 */

#include "lcd.h"
#include "main.h"

void delay(unsigned long int s){
	Delay_ms(s);
}

const uint8_t lcd_2x16_decode[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

void lcd_clrscr(){
	lcd_write_data(0b00000001);
}

void lcd_write_double_xx_xx(double data){
	uint16_t predata = (uint16_t) data;
	uint16_t postdata = (uint16_t) (data * 100) - predata * 100;
	lcd_write_dec_xx(predata);
	lcd_write_str(".");
	lcd_write_dec_xx(postdata);
}

void lcd_write_double_xxxx(double data){
	uint16_t predata = (uint16_t) data;
	lcd_write_dec_xxxx(predata);
}

void lcd_write_dec_xxxxx(uint16_t data){
	lcd_write_data(lcd_2x16_decode[(data / 10000) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 10000) / 1000) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[(((data % 10000) % 1000) / 100 ) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((((data % 10000) % 1000) % 100) / 10) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((((data % 10000) % 1000) % 100) % 10) & 0x0F]);
}

void lcd_write_dec_xxxx(uint16_t data){
	lcd_write_data(lcd_2x16_decode[(data / 1000) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 1000) / 100) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 1000) % 100) / 10 & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 1000) % 100) % 10 & 0x0F]);
}

void lcd_write_dec_xxx(uint16_t data){
	lcd_write_data(lcd_2x16_decode[(data / 100) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 100) / 10) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 100) % 10) & 0x0F]);
}

void lcd_write_dec_xx(uint8_t data){
	lcd_write_data(lcd_2x16_decode[((data % 100) / 10) & 0x0F]);
	lcd_write_data(lcd_2x16_decode[((data % 100) % 10) & 0x0F]);
}

void lcd_write_dec_x(uint8_t data) {
	lcd_write_data(lcd_2x16_decode[data]);
}

void lcd_write_dec_str_xxx_xx_angle(double data,int line, int pos, char *str){
	if (data < 0.0) {
		lcd_set_cursor(line,pos);
		lcd_write_str(str);
		lcd_set_cursor(line,pos+2);
		lcd_write_str("-");
		lcd_set_cursor(line,pos+3);
		lcd_write_dec_xx((-1)*data);
	 }
	 else {
		 lcd_set_cursor(line,pos);
		 lcd_write_str(str);
		 lcd_set_cursor(line,pos+2);
		 lcd_write_str("+");
		 lcd_set_cursor(line,pos+3);
		 lcd_write_dec_xx(data);
	 }
}

void lcd_write_dec_sign_xxxx(double data,int line, int pos){
	 if (data < 0.0) {
		 lcd_set_cursor(line,pos);
		 lcd_write_str("-");
		 lcd_set_cursor(line,pos+1);
		 lcd_write_dec_xxxx((-1)*data);
	 }
	 else {
		 lcd_set_cursor(line,pos);
		 lcd_write_str("+");
		 lcd_set_cursor(line,pos+1);
		 lcd_write_dec_xxxx(data);
	}
}

void lcd_init_gpio() {
	RCC_AHB1PeriphClockCmd(LCD_RCC_GPIO,ENABLE);

	GPIO_InitTypeDef init;
	init.GPIO_Mode = GPIO_Mode_OUT;
	init.GPIO_Pin =   GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4 |
					  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	init.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(LCD_PORT,&init);
}

void lcd_write_data(u16 data) {
	GPIO_SetBits(LCD_PORT, data);				// Put data
	GPIO_SetBits(LCD_PORT, LCD_E_Pin);			// EN = 1
	delay(1);									// wait at least 450 ns
	GPIO_ResetBits(LCD_PORT, LCD_E_Pin);		// EN = 0
	GPIO_ResetBits(LCD_PORT, data);				// EN = 0
	delay(1);									// wait 200 us for data
}

void lcd_init() {
	lcd_init_gpio();
	delay(40);								//wait 40 ms for start
	GPIO_ResetBits(LCD_PORT, LCD_RS_Pin);  	// rs = 0

	GPIO_SetBits(LCD_PORT, 0x30);			// init1
	delay(5);

	GPIO_SetBits(LCD_PORT, 0x30);			// init2
	delay(1);

	GPIO_SetBits(LCD_PORT, 0x30);
	delay(1);

	lcd_write_data(0b00111000); 			// function set  8bit 2line 5x8 dots

	lcd_write_data(0b00001100); 			// display on + cursor underline + blinking

	lcd_write_data(0b00000001); 			// clear

	lcd_write_data(0b00000110); 			// entry mode set

	lcd_write_data(0b00000010); 			// return to home

	GPIO_SetBits(LCD_PORT,LCD_RS_Pin);  	// rs = 1
}

void lcd_write_str(char*str) {
	do {
		lcd_write_data(*str);
	}while(*++str);
}

void lcd_write_cmd(u16 cmd) {
	GPIO_ResetBits(LCD_PORT,LCD_E_Pin);		// EN = 0
	GPIO_ResetBits(LCD_PORT, LCD_RS_Pin);	// RS = 0 for command

	GPIO_SetBits(LCD_PORT, LCD_E_Pin);		// EN = 1
	GPIO_SetBits(LCD_PORT, cmd);			// Put command
	delay(1);								// wait at least 450 ns
	GPIO_ResetBits(LCD_PORT, LCD_E_Pin);		// EN = 0
	GPIO_ResetBits(LCD_PORT, cmd);		// EN = 0
	delay(1);								// wait 5 ms for command
	GPIO_SetBits(LCD_PORT,LCD_RS_Pin);
}

void lcd_set_cursor(int line,int pos) {
	pos |= 0b10000000;
	if (line == 1) {
		pos += 0x40;
	}
	lcd_write_cmd(pos);
}
