/*
 * LTC_SPI.h
 *
 *  Created on: Dec 29, 2016
 *      Author: elenahuang
 */

#ifndef LTC_SPI_H_
#define LTC_SPI_H_

#include "stm32l4xx_hal.h"
#include "main.h"

void output_low();

void output_high();

int spi_send(uint8_t command);

int spi_recevie(uint8_t command);


#endif /* LTC_SPI_H_ */
