/*
 * LTC_SPI.c
 *
 *  Created on: Dec 29, 2016
 *      Author: elenahuang
 */

#include "LTC_SPI.h"
#include "stm32l4xx_hal.h"


extern SPI_HandleTypeDef hspi1;
uint16_t size = 8;



void ouput_low(){
	HAL_GPIO_WritePin(LTC_CS_GPIO_Port , LTC_CS_Pin, GPIO_PIN_RESET);
	//pull ltc6804_cs low
}

void output_high(){
	HAL_GPIO_WritePin(LTC_CS_GPIO_Port , LTC_CS_Pin, GPIO_PIN_SET);
	//pull ltc6804_cs high
}

int spi_send(uint8_t command){
	uint8_t TxBuffer[8];			//total_ic is one thus 8*total_ic = 8

	TxBuffer[0] = command;

	HAL_StatusTypeDef errorcode;
	errorcode = HAL_SPI_Transmit(&hspi1,TxBuffer,size,5000);	//timeout is 5 seconds(referenced from example spi code)

	return (int)errorcode;
	//0 - successful
	//1 - unsuccessful
	//2 - busy
}

int spi_receive(uint8_t command){


	uint8_t RxBuffer[8];			//total_ic is one thus 8*total_ic = 8

	RxBuffer[0] = command;


	HAL_StatusTypeDef errorcode;
	errorcode = HAL_SPI_Receive(&hspi1,RxBuffer,size,5000);	//timeout is 5 seconds(referenced from example spi code)


	return (int)errorcode;
	//0 - successful
	//1 - unsuccessful
	//2 - busy
}


