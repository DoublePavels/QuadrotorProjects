#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_spi.h"

#include "FreeRTOS.h"
#include "task.h"

#include "MPU6050.h"
#include "lcd.h"
#include "nRF24L01P.h"
#include "pwm.h"

#include <math.h>
#include "CompFilter.h"
#include <stdbool.h>
#include "main.h"


/*********************************************************************************/
/*---------------------------------- MEMS-SENSOR-VARIABLES-----------------------*/
/*********************************************************************************/
int16_t AccX;
int16_t AccY;
int16_t AccZ;

int16_t GyrX;
int16_t GyrY;
int16_t GyrZ;

int16_t Temp;

volatile double Ix = 0.0;     		/* Integral value GyrX */
volatile double dIx = 0.0;      	/* Diff Integral value GyrX */
volatile double Gx_Prev = 0.0;      /* Previous GyrX */
volatile double Gx_Cur = 0.0;       /* Current GyrX */

volatile double Iy = 0.0;      		/* Integral value GyrY */
volatile double dIy = 0.0;      	/* Integral value GyrY */
volatile double Gy_Prev = 0.0;      /* Previous GyrY */
volatile double Gy_Cur = 0.0;       /* Current GyrY */

volatile uint64_t t_Cur = 0;    	/* Value X(i-1) */
volatile uint64_t t_Prev = 0;   	/* Value X(i) */

volatile double h = 0.0;   			/* integration step */

volatile double ACCEL_XANGLE = 0.0;
volatile double ACCEL_YANGLE = 0.0;

volatile double Cx = 0.0;      		/* Angle X */
volatile double Cy = 0.0;      		/* Angle Y */

/*********************************************************************************/
/*-------------------------------nRF24l01-VARIABLES------------------------------*/
/*********************************************************************************/
unsigned char ADD[3] = {0};
unsigned char rx_buff[1];
volatile char received_byte;

/*********************************************************************************/
/*----------------------------------PWM-VARIABLES--------------------------------*/
/*********************************************************************************/
volatile bool 		pwm_1_incremented = false;
volatile bool 		pwm_2_incremented = false;
volatile bool 		pwm_3_incremented = false;
volatile bool 		pwm_4_incremented = false;

volatile bool 		pwm_1_decremented = false;
volatile bool 		pwm_2_decremented = false;
volatile bool 		pwm_3_decremented = false;
volatile bool 		pwm_4_decremented = false;

volatile uint16_t 	pwm_1_pulse = 120;
volatile uint16_t 	pwm_2_pulse = 120;
volatile uint16_t 	pwm_3_pulse = 120;
volatile uint16_t 	pwm_4_pulse = 120;

/*********************************************************************************/
/*--------------------------------------FUNCTIONS--------------------------------*/
/*********************************************************************************/
void Delay_ms(uint16_t milisec)
{
	 portTickType xLastWakeTime;
	 const portTickType xFrequency = portTICK_PERIOD_MS * milisec;
	  /* Initialise the xLastWakeTime variable with the current time. */
	 xLastWakeTime = xTaskGetTickCount ();
	 vTaskDelayUntil( &xLastWakeTime, xFrequency );
}

/*********************************************************************************/
/*----------------------------------------TASKS----------------------------------*/
/*********************************************************************************/
void vTaskLED1(void *pvParameters) {
	for (;;) {
	}
}

void vTaskPWM(void *pvParameters) {
	InitializeLEDs();
	InitializeTimer();
	InitializePWMChannel_1(pwm_1_pulse);
	InitializePWMChannel_2(pwm_2_pulse);
	InitializePWMChannel_3(pwm_3_pulse);
	InitializePWMChannel_4(pwm_4_pulse);
	for (;;) {
		if (pwm_1_incremented) {
			pwm_1_pulse += 1;
			InitializePWMChannel_1(pwm_1_pulse);
			pwm_1_incremented = false;
		}
		/* else */ if (pwm_2_incremented) {
			pwm_2_pulse += 1;
			InitializePWMChannel_2(pwm_2_pulse);
			pwm_2_incremented = false;
		}
		else if (pwm_3_incremented) {
			pwm_3_pulse += 1;
			InitializePWMChannel_3(pwm_3_pulse);
			pwm_3_incremented = false;
		}
		else if (pwm_4_incremented) {
			pwm_4_pulse += 1;
			InitializePWMChannel_4(pwm_4_pulse);
			pwm_4_incremented = false;
		}
		else if (pwm_1_decremented) {
			pwm_1_pulse -= 1;
			InitializePWMChannel_1(pwm_1_pulse);
			pwm_1_decremented = false;
		}
		else if (pwm_2_decremented) {
			pwm_2_pulse -= 1;
			InitializePWMChannel_2(pwm_2_pulse);
			pwm_2_decremented = false;
		}
		else if (pwm_3_decremented) {
			pwm_3_pulse -= 1;
			InitializePWMChannel_3(pwm_3_pulse);
			pwm_3_decremented = false;
		}
		else if (pwm_4_decremented) {
			pwm_4_pulse -= 1;
			InitializePWMChannel_4(pwm_4_pulse);
			pwm_4_decremented = false;
		}
	}
}

void vTaskLCD(void *pvParameters) {
	lcd_init();

	received_byte = 0;
	for (;;) {
		//lcd_write_dec_str_xxx_xx_angle(Cx,0,0,"Cx");           	/*  Pitch Angle by X */
		//lcd_write_dec_str_xxx_xx_angle(ACCEL_XANGLE,0,5,"Ax"); 	/*  Pitch Angle by X */

		//lcd_write_dec_str_xxx_xx_angle(Cy,1,0,"Cy");           	/*  Yaw Angle by Y */
		//lcd_write_dec_str_xxx_xx_angle(ACCEL_YANGLE,1,5,"Ay"); 	/*  Yaw Angle by Y */

		/* Temperature */
		//lcd_write_dec_str_xxx_xx_angle(Temp,1,11,"tC");

		/* PWM */
		lcd_set_cursor(0,0);
		lcd_write_dec_xxx(pwm_1_pulse);
		lcd_set_cursor(0,4);
		lcd_write_dec_xxx(pwm_2_pulse);
		lcd_set_cursor(1,0);
		lcd_write_dec_xxx(pwm_3_pulse);
		lcd_set_cursor(1,4);
		lcd_write_dec_xxx(pwm_4_pulse);
		/* Received data from nRF24l01 */
		if (received_byte <= 0x0F) {
			lcd_set_cursor(0,11);
			lcd_write_str("RCV:");
			lcd_set_cursor(0,14);
			lcd_write_dec_xx(received_byte);
		}
		else {
			lcd_set_cursor(0,0);
			lcd_write_str("RCV:__");
			lcd_set_cursor(0,14);
		}
	}
}

void vTaskMEMS(void *pvParameters) {
	MPU6050_I2C_Init();
	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	MPU6050_Initialize();
	GPIO_ResetBits(GPIOD, GPIO_Pin_15);

	if( MPU6050_TestConnection()== SUCCESS) {
		/* connection success */
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
	}

	MPU6050_SetFullScaleGyroRange(0);

	for (;;) {
		s16 AccelGyro[7] = {0};
		MPU6050_GetRawAccelGyro(AccelGyro);

		AccX = AccelGyro[0];
		AccY = AccelGyro[1];
		AccZ = AccelGyro[2];

		GyrX = AccelGyro[3];
		GyrY = AccelGyro[4];
		GyrZ = AccelGyro[5];

		Temp = (double)AccelGyro[6] / 340.0 + 36.53;

		ACCEL_XANGLE = 57.295 * atan( (float)AccY / sqrt( pow( (float)AccZ, 2 ) + pow( (float)AccX, 2 )));
		ACCEL_YANGLE = 57.295 * atan( (float)-AccX / sqrt( pow( (float)AccZ, 2 ) + pow( (float)AccY, 2 )));

		Gx_Cur = (float)GyrX / 131.0 + 0.65; 	/* Calibration */
		Gy_Cur = (float)GyrY / 131.0 - 2.7; 	/* Calibration */
		t_Cur  = xTaskGetTickCount();

		Cx = GetPitchAngle(Cx, Gx_Cur, Gx_Prev, t_Cur, t_Prev, AccX, AccY, AccZ);
		Cy = GetYawAngle  (Cy, Gy_Cur, Gy_Prev, t_Cur, t_Prev, AccX, AccY, AccZ);

		t_Prev  = t_Cur;
		Gx_Prev = Gx_Cur;
		Gy_Prev = Gy_Cur;

		Delay_ms(1);
  }
}

void vTask_nRF24L01 (void* pdata){
	nRF24L01_HW_Init();
  	RX_Mode();
	for (;;) {
		if(nRF24L01_RxPacket(rx_buff)) {
			if (rx_buff[0] <= 0x0F) {
				received_byte = rx_buff[0];
				switch (rx_buff[0]) {

					case 0x01:
						pwm_1_incremented = true;
						break;

					case 0x02:
						pwm_2_incremented = true;
						break;

					case 0x03:
						pwm_3_incremented = true;
						break;

					case 0x0A:
						pwm_4_incremented = true;
						break;

					case 0x04:
						pwm_1_decremented = true;
						break;

					case 0x05:
						pwm_2_decremented = true;
						break;

					case 0x06:
						pwm_3_decremented = true;
						break;

					case 0x0B:
						pwm_4_decremented = true;
						break;
					default:
						break;
				}
			}
			else {
				received_byte = 0xFF;
			}
		(void)SPI_RD_Reg(0x17);
	  }
  }
}

/*********************************************************************************/
/*----------------------------------MAIN FUNCTION--------------------------------*/
/*********************************************************************************/
int main(void)
{
	    //xTaskCreate( vTaskLED1, "LED1", configMINIMAL_STACK_SIZE, (void *) NULL, 1, NULL);

	    xTaskCreate( vTaskPWM, "PWM", configMINIMAL_STACK_SIZE, (void *) NULL, 2, NULL);

	    xTaskCreate( vTaskLCD, "LCD", configMINIMAL_STACK_SIZE, (void *) NULL, 2, NULL);

	    //xTaskCreate( vTaskMEMS, "MEMS", configMINIMAL_STACK_SIZE, (void *) NULL, 2, NULL);

	    xTaskCreate( vTask_nRF24L01, "nRF24L01", configMINIMAL_STACK_SIZE, (void *) NULL, 4, NULL);

	    vTaskStartScheduler();

	    while(1);
#if 0
	    InitializeLEDs();
	    InitializeTimer();
	    InitializePWMChannel_1(pwm_1_pulse);
	    InitializePWMChannel_2(pwm_2_pulse);
	    InitializePWMChannel_3(pwm_3_pulse);
	    InitializePWMChannel_4(pwm_4_pulse);
#endif
}

/*********************************************************************************/
/*-----------------------------------END-OF-FILE---------------------------------*/
/*********************************************************************************/
