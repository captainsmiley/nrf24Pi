/*
 * nrf24.h
 *
 *  Created on: Jul 28, 2016
 *      Author: tobias
 */

#ifndef NRF24_H_
#define NRF24_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include "bcm2835.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <iostream>
#include <string>

// nRF24L01P registers we need
#define _NRF24_CONFIG      0x00
#define _NRF24_EN_AA       0x01
#define _NRF24_EN_RXADDR   0x02
#define _NRF24_SETUP_AW    0x04
#define _NRF24_SETUP_RETR  0x03
#define _NRF24_RF_CH       0x05
#define _NRF24_RF_SETUP    0x06
#define _NRF24_STATUS      0x07
#define _NRF24_OBSERVE_TX  0x08
#define _NRF24_CD         0x09
#define _NRF24_TX_ADDR     0x10
#define _NRF24_RX_ADDR_P0  0x0A
#define _NRF24_RX_ADDR_P1  0x0B
#define _NRF24_RX_ADDR_P2  0x0C
#define _NRF24_RX_ADDR_P3  0x0D
#define _NRF24_RX_ADDR_P4  0x0E
#define _NRF24_RX_ADDR_P5  0x0F

#define _NRF24_RX_PW_P0  0x11
#define _NRF24_RX_PW_P1  0x12
#define _NRF24_RX_PW_P2  0x13
#define _NRF24_RX_PW_P3  0x14
#define _NRF24_RX_PW_P4 0x15
#define _NRF24_RX_PW_P5 0x16

#define _NRF24_FIFO_STATUS 0x17

#define R_RX_PAYLOAD 0x61

#define byte unsigned char
#define NOP           0xFF
#define R_REGISTER    0x00
#define REGISTER_MASK 0x1F
#define W_REGISTER    0x20
#define FLUSH_TX	  0xE1
#define FLUSH_RX	  0xE2




void nrf24_int_handler();

class nrf24 {
public:
	enum Mode { RX, TX, OFF };
	nrf24();
	void printStatus();
	virtual ~nrf24();
	void ceLow(void);
	void ceHigh(void);
	uint8_t getStatus();
	void readFIFO(byte * buff, uint8_t nr_bytes);
	void clearInt_RX_DR();
	void setup(Mode mode);
	void flushRx();
	void flushTx();
private:

	void setupRx();
	void irq_callback();
	byte getRegister(uint8_t reg);
	void setRegister(uint8_t reg, uint8_t value);
	void printConfig();
	byte * getAddress(byte * address_buf,byte typ,byte number_of_bytes);
	void set_address_with(uint8_t aw);
	void setRX(void);
	void powerDown(void);


	void powerUp(void);

	void printAddresses();

	void printRegister(uint8_t reg);

	std::string getRegName(uint8_t reg);

	//SPI buffer (payload max 32 bytes)
	uint8_t spi_buff[33] ;
	uint8_t ce_pin = 6; //25
	uint8_t csn_pin =10 ; //8
protected:
	static nrf24 * active_nrf24;

	friend void nrf24_int_handler();
};


#endif /* NRF24_H_ */
