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


//pull cs low start communication
void output_low(){
	HAL_GPIO_WritePin(LTC_CS_GPIO_Port , LTC_CS_Pin, GPIO_PIN_RESET);
}


//pull cs high end communication
void output_high(){
	HAL_GPIO_WritePin(LTC_CS_GPIO_Port , LTC_CS_Pin, GPIO_PIN_SET);
}


void spi_TransmitReceive(uint8_t*cmd,uint8_t size){
	HAL_StatusTypeDef errorcode;
	errorcode = HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)TxBuffer,(uint8_t*)RxBuffer,4,5000);

	if(errorcode == HAL_OK)	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

}
