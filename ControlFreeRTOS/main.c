#include "FreeRTOS.h"
#include "task.h"
/*---------------------------- Include ---------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "nRF24L01P.h"
#include "KeyBoard_lib.h"
#include "main.h"
#include <math.h>


char str[64];

volatile u8 button;
unsigned char i;
unsigned char ADD[3]={0x01,0x01,0x01};
unsigned char tx_buff[TX_PLOAD_WIDTH]={0};
unsigned char rx_buff[RX_PLOAD_WIDTH]={0};
static __IO uint32_t TimingDelay;
unsigned char Tx_Buf[1];


/* Private define ------------------------------------------------------------*/
#define CONFIG          0x00  //  Configurate the status of transceiver, mode of CRC and the replay of transceiver status
#define WRITE_nRF_REG   0x20  //  Command for write register




/* Private function prototypes -----------------------------------------------*/
//void Delay_ms(uint16_t nTime);
//void TimingDelay_Decrement(void);

/* Private functions ---------------------------------------------------------*/

void initGPIO() {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void Delay_ms(uint16_t milisec)
{
	 portTickType xLastWakeTime;
	 const portTickType xFrequency = portTICK_PERIOD_MS * milisec;
	  // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount ();
	 vTaskDelayUntil( &xLastWakeTime, xFrequency );
}
void vTask_Button (void* pdata){
  for (;;) {
	  button = getButton();
  }
}

void vTask_nRF24L01 (void* pdata){
	nRF24L01_HW_Init();
	TX_Mode();
	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x38); // enable power up and ptx
	while (1) {
		if (button == 0xFFU) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
		}
		else {
			Tx_Buf[0] = button;
			nRF24L01_TxPacket(Tx_Buf);

				GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
				switch (Tx_Buf[0]) {
				case 0x01U:
					GPIO_SetBits  (GPIOD, GPIO_Pin_12);
					break;

				case 0x02U:
					GPIO_SetBits  (GPIOD, GPIO_Pin_13);
					break;

				case 0x03U:
					GPIO_SetBits  (GPIOD, GPIO_Pin_14);
					break;

				case 0x04U:
					GPIO_SetBits  (GPIOD, GPIO_Pin_15);
					break;

				default:
					GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
					break;
				}

				Delay_ms(10);
		}
	}
}


int main(void)
{
	SystemInit();
	initGPIO();

	xTaskCreate(vTask_nRF24L01, "nRF24L01", configMINIMAL_STACK_SIZE, (void *) NULL, 2, NULL);
	xTaskCreate(vTask_Button, "Button", configMINIMAL_STACK_SIZE, (void *) NULL, 2, NULL);

	vTaskStartScheduler();

    while(1)
    {
    }
}

