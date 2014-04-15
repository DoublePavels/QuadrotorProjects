#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "nRF24l01P.h"
#include "main.h"

//define the private constants in this library
//#define TX_ADR_WIDTH 5
//#define RX_ADR_WIDTH 5

unsigned char TxBuf[Buffer_Size] = {0};
unsigned char RxBuf[Buffer_Size] = {0};


//define the initial Address

unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
// unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xB3,0xB4,0xB5,0xB6,0xF1};
// unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xB3,0xB4,0xB5,0xB6,0xF1};
// unsigned char  TX_ADDRESS[ADR_WIDTH]= {0x01,0x23,0x45,0x67,0x89};
// unsigned char  RX_ADDRESS[ADR_WIDTH]= {0x01,0x23,0x45,0x67,0x89};
//Define the layer1:HW operation
unsigned char nRF24L01_SPI_Send_Byte(unsigned char dat);
void nRF24L01_HW_Init(void);
void nRF24L01_SPI_NSS_L(void);
void nRF24L01_SPI_NSS_H(void);
void nRF24L01_CE_L(void);
void nRF24L01_CE_H(void);
//Define the layer2:Reg operation
unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_RD_Reg(unsigned char reg);
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len);
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len);

//Define the other function
void nRF24L01_Delay_10us(unsigned long n);

//Define the layer3 functions
void RX_Mode(void)
{
  nRF24L01_CE_L();
	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x39);
	nRF24L01_Delay_10us(2);
	SPI_WR_Reg(WRITE_nRF_REG + SETUP_AW, 0x03); 						// Setup add width 5 bytes
	nRF24L01_Delay_10us(2);
	SPI_WR_Reg(WRITE_nRF_REG + RF_CH,0x02);								// Setup frequency
	nRF24L01_Delay_10us(2);
	SPI_WR_Reg(WRITE_nRF_REG + RF_SETUP,  0x07);						// Setup power and rate
	nRF24L01_Delay_10us(2);
	SPI_WR_Reg(WRITE_nRF_REG + RX_PW_P0,1); 							// Number of bytes in data P0
	nRF24L01_Delay_10us(2);
	SPI_WR_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01); 						// Enable data P0
	nRF24L01_Delay_10us(2);
	SPI_Write_Buf(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH); 		// Write address into tx_add
	nRF24L01_Delay_10us(2);
	SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH); 	// Write address into rx_add_p0
	nRF24L01_Delay_10us(2);
	SPI_WR_Reg(WRITE_nRF_REG + EN_AA, 0x00);     						// Disable auto-ack for all channels
	nRF24L01_Delay_10us(2);
	
	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x33); 							// Enable power up and prx
	nRF24L01_Delay_10us(2);
	nRF24L01_CE_H();
	nRF24L01_Delay_10us(200);
	//nRF24L01_CE_L();
}

void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, TX_ADDRESS, ADR_WIDTH);
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
}

unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
	unsigned char flag = 0;
	unsigned char status;
    //if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)==0) {
		status=SPI_RD_Reg(NRFRegSTATUS);
		nRF24L01_CE_L();
		nRF24L01_Delay_10us(2);
		if(status & 0x40) 									//Data Ready RX FIFO interrupt
		{
			SPI_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);
			flag =1;
		}
		nRF24L01_Delay_10us(2);
		SPI_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS, 0x40); 		// Write 1 to clear bit
		nRF24L01_Delay_10us(2);
		nRF24L01_SPI_NSS_L();  
		nRF24L01_SPI_Send_Byte(0xE2);						// Flush RX FIFO
		nRF24L01_SPI_NSS_H();  
		nRF24L01_Delay_10us(2);
		SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x33); 			// enable power up and prx
		nRF24L01_CE_H();
   //}
	//SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x33); 			// enable power up and prx
	nRF24L01_Delay_10us(200);
	return flag;
}

//Define the layer2 functions
unsigned char SPI_RD_Reg(unsigned char reg)
{
	unsigned char reg_val;

	nRF24L01_SPI_NSS_L();               		 	// CSN low, initialize SPI communication...
	nRF24L01_Delay_10us(2);
	nRF24L01_SPI_Send_Byte(reg);            		// Select register to read from..
	reg_val = nRF24L01_SPI_Send_Byte(0);    		// ..then read register value
	nRF24L01_SPI_NSS_H();                			// CSN high, terminate SPI communication

	return(reg_val);        						// return register value
}

unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;

	nRF24L01_SPI_NSS_L();                  			// CSN low, init SPI transaction
	nRF24L01_Delay_10us(2);
	status = nRF24L01_SPI_Send_Byte(reg);			// select register
	nRF24L01_SPI_Send_Byte(value);             		// ..and write value to it..
	nRF24L01_SPI_NSS_H();                   		// CSN high again

	return(status);            						// return nRF24L01 status unsigned char
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
	unsigned int status,i;

	nRF24L01_SPI_NSS_L();                  // Set CSN low, init SPI tranaction
	nRF24L01_Delay_10us(2);
	status = nRF24L01_SPI_Send_Byte(reg);  // Select register to write to and read status unsigned char

  for(i=0; i < Len; i++)
  {
     pBuf[i] = nRF24L01_SPI_Send_Byte(0);
  }

	nRF24L01_SPI_NSS_H();

	return(status);                    // return nRF24L01 status unsigned char
}

unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
	unsigned int status,i;

	nRF24L01_SPI_NSS_L();
	nRF24L01_Delay_10us(2);
	status = nRF24L01_SPI_Send_Byte(reg);
	for(i=0; i<Len; i++)
	{
		nRF24L01_SPI_Send_Byte(*pBuf);
		pBuf ++;
	}
	nRF24L01_SPI_NSS_H();    
	return(status);   
}



//Define the layer1 functions
unsigned char nRF24L01_SPI_Send_Byte(unsigned char dat)
{
  /* Loop while DR register in not emplty */
  while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPIx peripheral */
  SPI_I2S_SendData(SPI, dat);

  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI);
}

void nRF24L01_SPI_NSS_H(void)
{
	GPIO_SetBits(GPIO_CS_CE, GPIO_Pin_CS);
}

void nRF24L01_SPI_NSS_L(void)
{
	GPIO_ResetBits(GPIO_CS_CE, GPIO_Pin_CS);
}

void nRF24L01_CE_L(void)
{
	GPIO_ResetBits(GPIO_CS_CE, GPIO_Pin_CE);
}

void nRF24L01_CE_H(void)
{
	GPIO_SetBits(GPIO_CS_CE, GPIO_Pin_CE);
}

void nRF24L01_HW_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_SPI, ENABLE);
	/* SPI Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APBPeriph_SPI, ENABLE);
	
	/* Connect SPI pins to AF */
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_CS_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_SCK_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MOSI_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MISO_SOURCE,GPIO_AF_SPI1);
	
	/* Configure SPI pins:  SCK ,MOSI, MISO*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SCK | GPIO_Pin_SPI_MOSI | GPIO_Pin_SPI_MISO;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	
	GPIO_Init(GPIO_SPI, &GPIO_InitStructure);

	/* Enable GPIO of CHIP SELECT */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_CS_CE, ENABLE);
	/* Configure CS pin */
	SPI_SSOutputCmd(SPI, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CS|GPIO_Pin_CE;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init(GPIO_CS_CE, &GPIO_InitStructure);
	
	/* Enable GPIO of IRQ */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure CS pin */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 84000kHz/256=328kHz < 400kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI, &SPI_InitStructure);

	SPI_CalculateCRC(SPI, DISABLE);

	/* SPI1 enable */
	SPI_Cmd(SPI, ENABLE);
}

void nRF24L01_Delay_10us(unsigned long n)
{
	Delay_10_ms(1);
}
