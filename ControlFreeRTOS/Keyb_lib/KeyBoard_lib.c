#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "KeyBoard_lib.h"
#include "lcd.h"

unsigned char Buttons[4][4] = {{0x01U, 0x02U, 0x03U, 0x0AU},
							   {0x04U, 0x05U, 0x06U, 0x0BU},
							   {0x07U, 0x08U, 0x09U, 0x0CU},
							   {0x0EU, 0x00U, 0x0FU, 0x0DU}};

GPIO_InitTypeDef GPIO_InitStructure;

void InitKBPinsForIn()
{
	  RCC_AHB1PeriphClockCmd(KB_RCC_Bus, ENABLE);

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	  GPIO_InitStructure.GPIO_Pin = KB_CON_1_PIN;
	  GPIO_Init(KB_CON_1_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_2_PIN;
	  GPIO_Init(KB_CON_2_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_3_PIN;
	  GPIO_Init(KB_CON_3_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_4_PIN;
	  GPIO_Init(KB_CON_4_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_5_PIN;
	  GPIO_Init(KB_CON_5_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_6_PIN;
	  GPIO_Init(KB_CON_6_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_7_PIN;
	  GPIO_Init(KB_CON_7_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = KB_CON_8_PIN;
	  GPIO_Init(KB_CON_8_PORT, &GPIO_InitStructure);
}

void KBRow_1_Out_H()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_1_PIN;
	  GPIO_Init(KB_ROW_1_PORT, &GPIO_InitStructure);
	  GPIO_SetBits(KB_ROW_1_PORT, KB_ROW_1_PIN);
}

void KBRow_1_Out_L()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_1_PIN;
	  GPIO_Init(KB_ROW_1_PORT, &GPIO_InitStructure);
	  GPIO_ResetBits(KB_ROW_1_PORT, KB_ROW_1_PIN);
}

void KBRow_2_Out_H()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_2_PIN;
	  GPIO_Init(KB_ROW_2_PORT, &GPIO_InitStructure);
	  GPIO_SetBits(KB_ROW_2_PORT, KB_ROW_2_PIN);
}

void KBRow_2_Out_L()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_1_PIN;
	  GPIO_Init(KB_ROW_2_PORT, &GPIO_InitStructure);
	  GPIO_ResetBits(KB_ROW_2_PORT, KB_ROW_2_PIN);
}

void KBRow_3_Out_H()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_3_PIN;
	  GPIO_Init(KB_ROW_3_PORT, &GPIO_InitStructure);
	  GPIO_SetBits(KB_ROW_3_PORT, KB_ROW_3_PIN);
}

void KBRow_3_Out_L()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_3_PIN;
	  GPIO_Init(KB_ROW_3_PORT, &GPIO_InitStructure);
	  GPIO_ResetBits(KB_ROW_3_PORT, KB_ROW_3_PIN);
}

void KBRow_4_Out_H()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_4_PIN;
	  GPIO_Init(KB_ROW_4_PORT, &GPIO_InitStructure);
	  GPIO_SetBits(KB_ROW_4_PORT, KB_ROW_4_PIN);
}

void KBRow_4_Out_L()
{
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = KB_ROW_4_PIN;
	  GPIO_Init(KB_ROW_4_PORT, &GPIO_InitStructure);
	  GPIO_ResetBits(KB_ROW_4_PORT, KB_ROW_4_PIN);
}

unsigned char getButton()
{
	// Check if 1-st row
	InitKBPinsForIn();
	KBRow_1_Out_H();
	if (GPIO_ReadInputDataBit(KB_CLMN_1_PORT, KB_CLMN_1_PIN) == 1) {
		delay(50);
		KBRow_1_Out_L();
		return Buttons[0][0];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_2_PORT, KB_CLMN_2_PIN) == 1) {
		delay(50);
		KBRow_1_Out_L();
		return Buttons[0][1];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_3_PORT, KB_CLMN_3_PIN) == 1) {
		delay(50);
		KBRow_1_Out_L();
		return Buttons[0][2];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_4_PORT, KB_CLMN_4_PIN) == 1) {
		delay(50);
		KBRow_1_Out_L();
		return Buttons[0][3];
	}
	KBRow_1_Out_L();

	// Check if 2-nd row
	InitKBPinsForIn();
	KBRow_2_Out_H();
	if (GPIO_ReadInputDataBit(KB_CLMN_1_PORT, KB_CLMN_1_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[1][0];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_2_PORT, KB_CLMN_2_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[1][1];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_3_PORT, KB_CLMN_3_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[1][2];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_4_PORT, KB_CLMN_4_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[1][3];
	}
	KBRow_2_Out_L();

	// Check if 3-rd row
	InitKBPinsForIn();
	KBRow_3_Out_H();
	if (GPIO_ReadInputDataBit(KB_CLMN_1_PORT, KB_CLMN_1_PIN) == 1) {
		delay(50);
		KBRow_3_Out_L();
		return Buttons[2][0];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_2_PORT, KB_CLMN_2_PIN) == 1) {
		delay(50);
		KBRow_3_Out_L();
		return Buttons[2][1];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_3_PORT, KB_CLMN_3_PIN) == 1) {
		delay(50);
		KBRow_3_Out_L();
		return Buttons[2][2];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_4_PORT, KB_CLMN_4_PIN) == 1) {
		delay(50);
		KBRow_3_Out_L();
		return Buttons[2][3];
	}
	KBRow_3_Out_L();


	// Check if 4-th row
	InitKBPinsForIn();
	KBRow_4_Out_H();
	if (GPIO_ReadInputDataBit(KB_CLMN_1_PORT, KB_CLMN_1_PIN) == 1) {
		delay(50);
		KBRow_4_Out_L();
		return Buttons[3][0];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_2_PORT, KB_CLMN_2_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[3][1];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_3_PORT, KB_CLMN_3_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[3][2];
	}
	else if (GPIO_ReadInputDataBit(KB_CLMN_4_PORT, KB_CLMN_4_PIN) == 1) {
		delay(50);
		KBRow_2_Out_L();
		return Buttons[3][3];
	}
	KBRow_4_Out_L();
	return 0xFFU;
}
