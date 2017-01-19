/*
 * LTC_SPI.h
 *
 *  Created on: Dec 29, 2016
 *      Author: elenahuang
 */

#ifndef LTC_SPI_H_
#define LTC_SPI_H_

#include "stm32l4xx_hal.h"
//#include "main.h"

#define delayUS_ASM(us) do {\
asm volatile ( "MOV R0,%[loops]\n\t"\
"1: \n\t"\
"SUB R0, #1\n\t"\
"CMP R0, #0\n\t"\
"BNE 1b \n\t" : : [loops] "r" (20*us) : "memory"\
     );\
}while(0)
//This allows the user to delay for (us)
//set the number to 10 from 1-3
//set the number to 15 from 4-12
//set the number to 20 from 13-3000


void output_low();

void output_high();

void spi_TransmitReceive(uint8_t*cmd);

#endif
