/*
 * Defining Keyboard Ports and pin connectors
 * CON - connector pin number 1..8
 */
#define KB_RCC_Bus		RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE

#define KB_CON_1_PORT	GPIOE
#define KB_CON_1_PIN	GPIO_Pin_11
#define KB_CON_2_PORT	GPIOE
#define KB_CON_2_PIN	GPIO_Pin_13
#define KB_CON_3_PORT	GPIOE
#define KB_CON_3_PIN	GPIO_Pin_15
#define KB_CON_4_PORT	GPIOB
#define KB_CON_4_PIN	GPIO_Pin_11
#define KB_CON_5_PORT	GPIOB
#define KB_CON_5_PIN	GPIO_Pin_13
#define KB_CON_6_PORT	GPIOB
#define KB_CON_6_PIN	GPIO_Pin_15
#define KB_CON_7_PORT	GPIOD
#define KB_CON_7_PIN	GPIO_Pin_9
#define KB_CON_8_PORT	GPIOD
#define KB_CON_8_PIN	GPIO_Pin_11

/*
 * ROW - line number of buttons 1(1,2,3,A)..4(*,0,#,D)
 */
#define KB_ROW_1_PORT	KB_CON_8_PORT
#define KB_ROW_1_PIN	KB_CON_8_PIN
#define KB_ROW_2_PORT	KB_CON_7_PORT
#define KB_ROW_2_PIN	KB_CON_7_PIN
#define KB_ROW_3_PORT	KB_CON_6_PORT
#define KB_ROW_3_PIN	KB_CON_6_PIN
#define KB_ROW_4_PORT	KB_CON_5_PORT
#define KB_ROW_4_PIN	KB_CON_5_PIN

/*
 * CLMN - column of buttons 1(1,4,7,*)..4(A,B,C,D)
 */
#define KB_CLMN_1_PORT	KB_CON_4_PORT
#define KB_CLMN_1_PIN	KB_CON_4_PIN
#define KB_CLMN_2_PORT	KB_CON_3_PORT
#define KB_CLMN_2_PIN	KB_CON_3_PIN
#define KB_CLMN_3_PORT	KB_CON_2_PORT
#define KB_CLMN_3_PIN	KB_CON_2_PIN
#define KB_CLMN_4_PORT	KB_CON_1_PORT
#define KB_CLMN_4_PIN	KB_CON_1_PIN

unsigned char getButton();
