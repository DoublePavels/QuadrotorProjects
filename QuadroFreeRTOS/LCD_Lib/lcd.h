/*
 * lcd.h
 *
 *  Created on: 23.05.2012
 *      Author: lamazavr
 */

#ifndef LCD_H_
#define LCD_H_
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <core_cm4.h>

#define LCD_PORT GPIOC
#define LCD_RCC_GPIO RCC_AHB1Periph_GPIOC
#define LCD_E_Pin GPIO_Pin_8
#define LCD_RS_Pin GPIO_Pin_9

FunctionalState curstate;
void lcd_clrscr();
void delay(unsigned long int s);
void lcd_init_gpio();
void lcd_write_data(u16 data);
void lcd_init();
void lcd_write_str(char*str);
void lcd_write_cmd(u16 cmd);
void lcd_set_cursor(int line,int pos);
void lcd_write_dec_xxxx(uint16_t data);
void lcd_write_dec_xxx(uint16_t data);
void lcd_write_dec_xx(uint8_t data);
void lcd_write_dec_x(uint8_t data);
void lcd_write_double_xx_xx(double data);
void lcd_write_double_xxxx(double data);
void lcd_write_dec_xxxxx(uint16_t data);
void lcd_write_dec_str_xxx_xx_angle(double data,int line, int pos, char *str);
void lcd_write_dec_sign_xxxx(double data,int line, int pos);
#endif /* LCD_H_ */
