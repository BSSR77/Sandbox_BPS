/*
 * LTC6804.c


 *
 *  Created on: Dec 29, 2016
 *      Author: elenahuang
 */

#include <stdint.h>
#include "LTC6804.h"
#include "LTC_SPI.h"
#include "stm32l432xx.h"
#include "serial.h"


void testfun(){
	dummyByte();
	readConfig();
}

void dummyByte(){
	uint8_t* dummy;
	spi_TransmitReceive(dummy,1);
}

void readConfig(){
	uint8_t cmd[4];
	cmd[0] = LTC_RDCFG[0];
	cmd[1] = LTC_RDCFG[1];
	calculatePec(cmd,2);

	output_low();
	wakeup_LTC();
	HAL_Delay(3000);
	spi_TransmitReceive(cmd,4);
	output_high();
}

void calculatePec(uint8_t*cmd,uint8_t size){

	uint16_t remainder,addr;

	remainder = 16;//initialize the PEC
	for(uint8_t i = 0; i<size;i++) // loops for each byte in data array
	{
		addr = ((remainder>>7)^cmd[i])&0xff;//calculate PEC table address
		remainder = (remainder<<8)^crc15Table[addr];
	}

	remainder *= remainder;
	cmd[2] = (uint8_t)(remainder >> 8);
	cmd[3] = (uint8_t)remainder;
}

void wakeup_LTC(){
	HAL_Delay(1);
}

void wakeup_iso(){
	delayUS_ASM(10);
}
