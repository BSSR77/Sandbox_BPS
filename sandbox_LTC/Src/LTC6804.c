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

uint8_t ADCV[2];
uint16_t cell_codes[TOTAL_IC][12];
uint16_t aux_codes[TOTAL_IC][6];
uint8_t tx_cfg[TOTAL_IC][6];
uint8_t rx_cfg[TOTAL_IC][8];


void LTC6804_outloop(){
	LTC_initialize();
	init_cfg();
}

void LTC6804_inloop(){
	// TODO: Fix this part to properly respond to commands
#ifdef DEBUG
	run_command(1);
#endif
}

void LTC_initialize(){
	output_low();
	output_high();


	uint8_t md_bits;
	md_bits = (modeNormal & 0x02) >> 1;
	ADCV[0] = md_bits + 0x02;
	md_bits = (modeNormal & 0x01) << 7;
	ADCV[1] =  md_bits + 0x60 + (notdischarge<<4) + cellSelect_all;

}



void LTC_startadc(){

	uint8_t command[4];
	uint16_t temp_pec;

	//1
	command[0] = ADCV[0];
	command[1] = ADCV[1];

	//2
	temp_pec = pec15_calc(2, ADCV);
	command[2] = (uint8_t)(temp_pec >> 8);
	command[3] = (uint8_t)(temp_pec);

	//3
	LTC_wakeup_sleep (); //This will guarantee that the LTC6804 SPI port is awake. This command can be removed.

	//4
	output_low();
	spi_send(command,4);
	output_high();

}

void LTC_readReg(uint8_t reg,uint8_t total_ic,uint8_t *data){

	uint8_t cmd[4];
	uint16_t temp_pec;

	if (reg == 1)
	{
		cmd[1] = 0x04;
	}
	else if (reg == 2)
	{
		cmd[1] = 0x06;
	}
	else if (reg == 3)
	{
		cmd[1] = 0x08;
	}
	else if (reg == 4)
	{
		cmd[1] = 0x0A;
	}

	LTC_wakeup_sleep (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

	for (int current_ic = 0; current_ic<total_ic; current_ic++)
	{
		cmd[0] = 0x80 + (current_ic<<3); //Setting address
		temp_pec = pec15_calc(2, cmd);
		cmd[2] = (uint8_t)(temp_pec >> 8);
		cmd[3] = (uint8_t)(temp_pec);
		output_low();
		spi_send(cmd, 4);
		output_high();
	}
}


void LTC_writeConfig(uint8_t total_ic,uint8_t config[][6])
{
	const uint8_t BYTES_IN_REG = 6;
	//const uint8_t CMD_LEN = 4+(8*total_ic);
	uint8_t cmd[4];
	uint16_t temp_pec;
	uint8_t cmd_index; //command counter

	//cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
	//1
	cmd[0] = 0x00;
	cmd[1] = 0x01;
	cmd[2] = 0x3d;
	cmd[3] = 0x6e;

	//2
	cmd_index = 4;
	for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC6804 in stack,
	{
		for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each byte in the CFGR register
		{
			// i is the byte counter

			cmd[cmd_index] = config[current_ic-1][current_byte];    //adding the config data to the array to be sent
			cmd_index = cmd_index + 1;
		}
		//3
		temp_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);// calculating the PEC for each board
		cmd[cmd_index] = (uint8_t)(temp_pec >> 8);
		cmd[cmd_index + 1] = (uint8_t)temp_pec;
		cmd_index = cmd_index + 2;
	}

	//4
	LTC_wakeup_sleep ();                                //This will guarantee that the LTC6804 SPI port is awake.This command can be removed.
	//5
	for (int current_ic = 0; current_ic<total_ic; current_ic++)
	{
		cmd[0] = 0x80 + (current_ic<<3); //Setting address
		temp_pec = pec15_calc(2, cmd);
		cmd[2] = (uint8_t)(temp_pec >> 8);
		cmd[3] = (uint8_t)(temp_pec);
		output_low();
		spi_send(cmd,4);
    output_high();
	}
	//free(cmd);
}




void LTC_clearCell(){

	uint8_t cmd[4];
	uint16_t cmd_pec;

	//1
	cmd[0] = 0x07;
	cmd[1] = 0x11;

	//2
	cmd_pec = pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec );

	//3
	LTC_wakeup_sleep (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

	//4
	output_low();
	spi_send(cmd,4);
	output_high();
}





/*
void wakeup_idle()
{
  	output_low();
  	//delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
  	output_high();
}*/


void LTC_wakeup_sleep()
{
  	output_low();
  	//delayUS_ASM(300); // Twake = 100us, to start, wait for 3xTwake
  	output_high();
}


uint16_t pec15_calc(uint8_t len, uint8_t *data)
{
  	uint16_t remainder,addr;

  	remainder = 16;//initialize the PEC
  	for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  	{
    	addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    	remainder = (remainder<<8)^crc15Table[addr];
  	}
  	return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

void run_command(uint32_t cmd)
{
	if(cmd == 1){
		//Serial.println("transmit 'm' to quit");
		LTC_wakeup_sleep();

		LTC_writeConfig(TOTAL_IC,tx_cfg);
		while (1){

			LTC_wakeup_sleep();
			LTC_startADC();
			//delay(10);
			LTC_wakeup_sleep();
			print_cells();
			//delay(500);
		}
    } else{
    	static uint8_t message[] = "Incorrect Option\n";
    	Serial2_writeBuf(message);
    }
}

void init_cfg()
{
  for(int i = 0; i<TOTAL_IC;i++)
  {
    tx_cfg[i][0] = 0xFE;
    tx_cfg[i][1] = 0x00 ;
    tx_cfg[i][2] = 0x00 ;
    tx_cfg[i][3] = 0x00 ;
    tx_cfg[i][4] = 0x00 ;
    tx_cfg[i][5] = 0x00 ;
  }

}


void print_cells(){

	for (uint8_t current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
		static uint8_t message1[] = "IC";
    	Serial2_writeBuf(message1);

		Serial2_write(current_ic+1);	//in decimal

		for(uint8_t i=0; i<12; i++)
		{
			static uint8_t message2[] = " C";
	    	Serial2_writeBuf(message2);

			Serial2_write(i+1);

	    	static uint8_t message3[] = ":";
	    	Serial2_writeBuf(message3);

	    	static uint8_t message4[2];
	    	message4[0] = (cell_codes[current_ic][i]) & 0xff;	//remember to time by 0.0001,4
	    	message4[1] = (cell_codes[current_ic][i]) >> 8;
	    	Serial2_writeBuf(message4);

	    	message4[1] = (cell_codes[current_ic][i]) >> 8;
	    	static uint8_t message5[] = ",";
	    	Serial2_writeBuf(message5);
		}
	}

}


/*
void print_rxconfig()
{
	printf("Received Configuration ");
	for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
	{
		printf(" IC ");
		printf(current_ic+1,DEC);
		printf(": 0x");
		serial_print_hex(rx_cfg[current_ic][0]);
    	printf(", 0x");
    	serial_print_hex(rx_cfg[current_ic][1]);
    	printf(", 0x");
    	serial_print_hex(rx_cfg[current_ic][2]);
    	printf(", 0x");
    	serial_print_hex(rx_cfg[current_ic][3]);
    	printf(", 0x");
    	serial_print_hex(rx_cfg[current_ic][4]);
    	printf(", 0x");
    	serial_print_hex(rx_cfg[current_ic][5]);
    	prinf(", Received PEC: 0x");
    	serial_print_hex(rx_cfg[current_ic][6]);
    	prinf(", 0x");
    	serial_print_hex(rx_cfg[current_ic][7]);
    	prinf();
	}
	prinf();
}*/

/*
void serial_print_hex(uint8_t data)
{
    if (data< 16)
    {
    	prinf("0");
    	prinf((byte)data,HEX);
    }
    else
    	prinft((bypt)data,HEX);
}
*/
