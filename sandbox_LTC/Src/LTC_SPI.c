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
uint8_t RxBuffer[8];
uint8_t TxBuffer[8];


void clrFIFO(){
	while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)){
		uint32_t garbage = hspi1.Instance->DR;
	}
}



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



//int spi_send(uint8_t *data, uint8_t size){
////	output_low();
////	delayUS_ASM(10);				//wakeup the chip sleep_idle
//	uint8_t TxBuffer[8];			//total_ic is one thus 8*total_ic = 8
//
//	for(uint8_t i = 0; i<size; i++){
//		TxBuffer[i] = data[i];
//	}
//
//	HAL_StatusTypeDef errorcode;
//	errorcode = HAL_SPI_Transmit(&hspi1,TxBuffer,size,500);	//timeout is 5 seconds(referenced from example spi code)
//
////	output_high();
//
//	return (int)errorcode;
//	//0 - successful
//	//1 - unsuccessful
//	//2 - busy
//}
//
//
//
//int spi_receive(){
//	clrFIFO();
////	output_low();
////	delayUS_ASM(10);
//	HAL_StatusTypeDef errorcode;
//	errorcode = HAL_SPI_Receive(&hspi1,RxBuffer,size,500);	//timeout is 5 seconds(referenced from example spi code)
//
////	output_high();
//	return (int)errorcode;
//	//0 - successful
//	//1 - unsuccessful
//	//2 - busy
//}

int spi_send_receive(uint8_t *data, uint8_t size){

	clrFIFO();

	for(uint8_t i = 0; i<size; i++){
		TxBuffer[i] = data[i];
	}

	HAL_StatusTypeDef errorcode;
	errorcode = HAL_SPI_Receive(&hspi1,TxBuffer,RxBuffer,500);



	return (int)errorcode;

}
