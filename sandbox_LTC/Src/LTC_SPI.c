/*
 * LTC_SPI.c
 *
 *  Created on: Dec 29, 2016
 *      Author: elenahuang
 */

#include "LTC_SPI.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include "serial.h"


extern SPI_HandleTypeDef hspi1;
uint16_t size = 8;



void output_low(){
	HAL_GPIO_WritePin(LTC_CS_GPIO_Port , LTC_CS_Pin, GPIO_PIN_RESET);

	//uint8_t outputmsg1[] = "output_low success";
	//Serial2_writeBuf(outputmsg1);
	//pull ltc6804_cs low
}



void output_high(){
	while( hspi1.State == HAL_SPI_STATE_BUSY );  // wait xmission complete
	HAL_GPIO_WritePin(LTC_CS_GPIO_Port , LTC_CS_Pin, GPIO_PIN_SET);

	//uint8_t outputmsg2[] = "output_high success";
	//Serial2_writeBuf(outputmsg2);
	//pull ltc6804_cs high
}


