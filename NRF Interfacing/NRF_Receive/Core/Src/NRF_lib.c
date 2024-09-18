/*
 * NRF_lib.c
 *
 *  Created on: Sep 14, 2024
 *      Author: Tech-user
 */
#include "stm32f4xx_hal.h"
#include "NRF_lib.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT GPIOC
#define NRF24_CE_PIN  GPIO_PIN_1

#define NRF24_CS_PORT GPIOC
#define NRF24_CS_PIN  GPIO_PIN_2

void CS_Select (void)
{
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect(void)
{
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

void CE_Enable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable (void)
{
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}


// write a single byte to the particular register
void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

//write multiple bytes starting from a particular register
void nrf24_WriteReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

//read single byte from a particular register
uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data=0;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return data;
}

//read single byte from a particular register
uint8_t nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return data;
}

void NRF24_Init (void)
{
	//disable the chip before configuring the device
	CE_Disable();

	nrf24_WriteReg(CONFIG, 0);       //will be configured later
	nrf24_WriteReg(EN_AA, 0);        //No auto ack
	nrf24_WriteReg(EN_RXADDR, 0);    //not enabling any data pipe now
	nrf24_WriteReg(SETUP_AW, 0x03);  //setup TX/RX address width to 5 bytes
	nrf24_WriteReg(SETUP_RETR, 0);   //disable retransmit
	nrf24_WriteReg(RF_CH, 0);        //channel will be set later
	nrf24_WriteReg(RF_SETUP, 0x0E);  //set rate = 2 Mb/s, power = 0 dBm

	//Enable the chip after configuring the device
	CE_Enable();

}

///////////////////////////////////////////////////////////////////////////////end of setup

//setup TX mode
void NRF24_TxMode (uint8_t *Address, uint8_t channel)
{
	//disable the chip before configuring the device
	CE_Disable();

	nrf24_WriteReg(RF_CH, channel);   //select the channel
	nrf24_WriteReg_Multi(TX_ADDR, Address, 5);  //write tx address


	//power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config |= (1<<1);
	nrf24_WriteReg(CONFIG, config);

	//Enable the chip after configuring the device
	CE_Enable();
}

//transmit the data
uint8_t NRF24_Transmit(uint8_t *data)
{
	uint8_t cmdtosend = 0;

	//select device
	CS_Select();

	//payload command
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	//send payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	//unselect device
	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	if ((fifostatus & (1<<4)) && !(fifostatus & (1<<3)))
	{
		cmdtosend = FLUSH_TX;
		HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

		return 1;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////end of tx

//setup Rx mode
void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	//disable the chip before configuring the device
	CE_Disable();

	nrf24_WriteReg(RF_CH, channel);   //select the channel

	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr |= (1<<1);
	nrf24_WriteReg(EN_RXADDR, en_rxaddr);          //enable data pipe 1
	nrf24_WriteReg_Multi(RX_ADDR_P1, Address, 5);  //write rx pipe address

	nrf24_WriteReg(RX_PW_P1, 32);     //set size of payload to 32 byte

	//power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config |= (1<<1) | (1<<0);
	nrf24_WriteReg(CONFIG, config);

	//Enable the chip after configuring the device
	CE_Enable();
}

uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);

	if((status & (1<<6)) && status & (pipenum<<1))
	{
		nrf24_WriteReg(STATUS, (1<<6));            //clear bit
		return 1;
	}
	return 0;
}

//receive the data
uint8_t NRF24_Receive(uint8_t *data)
{
	uint8_t cmdtosend = 0;

	//select device
	CS_Select();

	//payload command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	//send payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	//unselect device
	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);
}






















