#include "stm32f4xx_rcc.h"

//Define the commands for operate the nRF24L01P
#define READ_nRF_REG    0x00  //  Command for read register
#define WRITE_nRF_REG   0x20  //  Command for write register
#define RD_RX_PLOAD     0x61  //  Command for read Rx payload
#define WR_TX_PLOAD     0xA0  //  Command for write Tx payload
#define FLUSH_TX        0xE1  //  Command for flush Tx FIFO
#define FLUSH_RX        0xE2  //  Command for flush Rx FIFO
#define REUSE_TX_PL     0xE3  //  Command for reuse Tx payload
#define NOP             0xFF  //  Reserve

//Define the register address for nRF24L01P
#define CONFIG          0x00  //  Configurate the status of transceiver, mode of CRC and the replay of transceiver status
#define EN_AA           0x01  //  Enable the atuo-ack in all channels
#define EN_RXADDR       0x02  //  Enable Rx Address
#define SETUP_AW        0x03  //  Configurate the address width
#define SETUP_RETR      0x04  //  Setup the retransmit
#define RF_CH           0x05  //  Configurate the RF frequency
#define RF_SETUP        0x06  //  Setup the rate of data, and transmit power
#define NRFRegSTATUS    0x07  //
#define OBSERVE_TX      0x08  //
#define CD              0x09  //  Carrier detect
#define RX_ADDR_P0      0x0A  //  Receive address of channel 0
#define RX_ADDR_P1      0x0B  //  Receive address of channel 1
#define RX_ADDR_P2      0x0C  //  Receive address of channel 2
#define RX_ADDR_P3      0x0D  //  Receive address of channel 3
#define RX_ADDR_P4      0x0E  //  Receive address of channel 4
#define RX_ADDR_P5      0x0F  //  Receive address of channel 5
#define TX_ADDR         0x10  //  Transmit address
#define RX_PW_P0        0x11  //  Size of receive data in channel 0
#define RX_PW_P1        0x12  //  Size of receive data in channel 1
#define RX_PW_P2        0x13  //  Size of receive data in channel 2
#define RX_PW_P3        0x14  //  Size of receive data in channel 3
#define RX_PW_P4        0x15  //  Size of receive data in channel 4
#define RX_PW_P5        0x16  //  Size of receive data in channel 5
#define FIFO_STATUS     0x17  //  FIFO Status
// define  GPIO for SPI

#define SPI                   		SPI1
#define GPIO_CS_CE                  GPIOA
#define GPIO_Pin_CE              	GPIO_Pin_3
#define GPIO_Pin_CS              	GPIO_Pin_4
#define RCC_AHB1Periph_GPIO_CS_CE	RCC_AHB1Periph_GPIOA


#define GPIO_SPI              		GPIOA
#define GPIO_Pin_SPI_SCK      		GPIO_Pin_5
#define GPIO_Pin_SPI_MISO     		GPIO_Pin_6
#define GPIO_Pin_SPI_MOSI     		GPIO_Pin_7
#define RCC_APBPeriph_SPI     		RCC_APB2Periph_SPI1
#define GPIO_Pin_SPI_CS_SOURCE      GPIO_PinSource4
#define GPIO_Pin_SPI_SCK_SOURCE     GPIO_PinSource5
#define GPIO_Pin_SPI_MISO_SOURCE    GPIO_PinSource6
#define GPIO_Pin_SPI_MOSI_SOURCE    GPIO_PinSource7
#define RCC_AHB1Periph_GPIO_SPI     RCC_AHB1Periph_GPIOA

#define nRF24l01_IRQ    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
///**************************************************************************************

#define ADR_WIDTH 5

#define RX_PLOAD_WIDTH 32
#define TX_PLOAD_WIDTH 32
#define Buffer_Size 32

//Define RF power value
#define P0dBm   0
#define Pm6dBm  1
#define Pm12dBm 2
#define Pm18dBm 3

//#define RF rate
#define R2mbps   0
#define R1mbps   1
#define R250kbps 3


unsigned char nRF24L01_Config(unsigned char freq,unsigned char power,unsigned char rate);
void RX_Mode(void);
void TX_Mode(void);
void nRF24L01_TxPacket(unsigned char * tx_buf);
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf);
void nRF24L01_HW_Init(void);
void nRF24L01_Init_Soft(void);
void nRF24L01_Set_TX_Address(unsigned char A,unsigned char B,unsigned char C,unsigned char D,unsigned char E);
void nRF24L01_Set_RX_Address(unsigned char A,unsigned char B,unsigned char C,unsigned char D,unsigned char E);
unsigned char SPI_RD_Reg(unsigned char reg);
unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value);

