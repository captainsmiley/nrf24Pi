/*
 * nrf24.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: tobias
 */

#include "nrf24.h"
#include <bitset>

nrf24::nrf24() {

	std::cout << "Running nrf24 constructor" << std::endl;




}

nrf24::~nrf24() {
	// TODO Auto-generated destructor stub
}

void nrf24::setup(Mode mode)
{
	//wiringPiSetup () ;
	wiringPiSPISetup (0, 2000000);
	pinMode(csn_pin,OUTPUT);
	pinMode(ce_pin,OUTPUT);
	digitalWrite (ce_pin, LOW);
	digitalWrite (csn_pin, HIGH);

	switch (mode)
	{
	case RX:
		setupRx();
		break;
	default:
		std::cout << "Invalid mode" << std::endl;
		break;
	}

}

void nrf24::setupRx()
{

	powerUp();
	setRegister(_NRF24_CONFIG,0b11);
	setRegister(_NRF24_EN_AA,0x00);
	setRegister(_NRF24_EN_RXADDR,0x01);
	setRegister(_NRF24_SETUP_AW,0b11);
	setRegister(_NRF24_RF_CH,0b10);

	setRegister(_NRF24_RX_PW_P0,0x01);


	printRegister(_NRF24_CONFIG);
	printRegister(_NRF24_EN_AA);
	printRegister(_NRF24_EN_RXADDR);
	printRegister(_NRF24_SETUP_AW);
	printRegister(_NRF24_SETUP_RETR);
	printRegister(_NRF24_RF_CH);
	printRegister(_NRF24_RF_SETUP);
	printRegister(_NRF24_STATUS);
	std::bitset<8> x(getStatus());
	std::cout << "Fast ststus gives: " << x << std::endl;
	printRegister(_NRF24_FIFO_STATUS);
	printAddresses();


	active_nrf24 =  this;
	wiringPiISR (0, INT_EDGE_FALLING,  &nrf24_int_handler) ;

	ceHigh();

}

nrf24 * nrf24::active_nrf24 = 0;


std::string nrf24::getRegName(uint8_t reg)
{
	switch(reg)
	{
	case _NRF24_CONFIG: return "CONFIG";
	break;
	case _NRF24_EN_AA: return "EN_AA";
	break;
	case _NRF24_EN_RXADDR: return "EN_RXADDR";
	break;
	case _NRF24_SETUP_AW: return "SETUP_AW";
	break;
	case _NRF24_SETUP_RETR: return "SETUP_RETR";
	break;
	case _NRF24_RF_CH: return "RF_CH";
	break;
	case _NRF24_RF_SETUP: return "RF_SETUP";
	break;
	case _NRF24_STATUS: return "STATUS";
	break;
	case _NRF24_OBSERVE_TX: return "OBSERVE_TX";
	break;
	case _NRF24_CD: return "CD";
	break;
	case _NRF24_FIFO_STATUS: return "FIFO_STATUS";
	break;
	}
	return "not_valid";
}

void nrf24::readFIFO(byte * buff, uint8_t nr_bytes)
{
	uint8_t * p = spi_buff;
	*p++ = ( R_RX_PAYLOAD );
	for(int i=0; i<nr_bytes;++i) *p++ = NOP ; // Dummy operation
	digitalWrite(csn_pin,LOW);
	wiringPiSPIDataRW (0, spi_buff, nr_bytes+1);
	digitalWrite(csn_pin,HIGH);
	for(int i=0; i<nr_bytes; ++i)
	{
		*(buff+i) = *(spi_buff+1+i);
	}

}

void nrf24::flushRx()
{
	*spi_buff = FLUSH_RX;
	digitalWrite(csn_pin,LOW);
	wiringPiSPIDataRW (0, spi_buff, 1);
	digitalWrite(csn_pin,HIGH);
}
void nrf24::flushTx()
{
	*spi_buff = FLUSH_TX;
	digitalWrite(csn_pin,LOW);
	wiringPiSPIDataRW (0, spi_buff, 1);
	digitalWrite(csn_pin,HIGH);
}

//nrf24 * active_nrf;
void nrf24_int_handler()
{
	//std::cout << "int handler exe" << std::endl;
	nrf24::active_nrf24->irq_callback();
}

void nrf24::irq_callback()
{
	delay(1);
	ceLow();
	if(getStatus() & 0b01000000)
	{
		byte a[32] = {};
		readFIFO(&a[1],1);
		std::cout << a[1] << std::flush;
		/*
			r.readFIFO(&a[1],1);
			std::cout << "Received: " << a[1] << std::endl;
			r.readFIFO(a,1);
			std::cout << "Received: " << a[1] << std::endl;
		 */
		clearInt_RX_DR();
		//printStatus();
	}
	ceHigh();
}

void nrf24::clearInt_RX_DR()
{
	setRegister(_NRF24_STATUS,(getStatus() | 0b01000000 ));
}

// get the value of a nRF24L01p register
uint8_t nrf24::getRegister(uint8_t reg)
{
uint8_t result;
uint8_t data[5];
  *data = ( R_REGISTER | ( REGISTER_MASK & reg ) );
  *(data+1) = NOP ; // Dummy operation, just for reading
  digitalWrite (csn_pin, LOW);
  wiringPiSPIDataRW (0, data, 2);
  digitalWrite (csn_pin, HIGH);
  result = *(data+1);   // result is 2nd byte of receive buffer
  return (result);
}

// set the value of a nRF24L01p register
void nrf24::setRegister(uint8_t reg, uint8_t value)
{
  uint8_t * prx = spi_buff;
  uint8_t * ptx = spi_buff;

 *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
 *ptx = value ;
  digitalWrite (10, LOW);
  wiringPiSPIDataRW (0, spi_buff, 2);
  digitalWrite (10, HIGH);
}
// power up the nRF24L01p chip
void nrf24::powerUp(void)
{
	ceLow();
	clearInt_RX_DR();
	flushRx();
	flushTx();
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x02);
  delayMicroseconds(130);
}

// switch nRF24L01p off
void nrf24::powerDown(void)
{
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)&~0x02);
}

// enable RX
void nrf24::ceHigh(void)
{
	digitalWrite(ce_pin,HIGH);
}

// disable RX
void nrf24::ceLow(void)
{
	digitalWrite(ce_pin,LOW);
}

// setup RX-Mode of nRF24L01p
void nrf24::setRX(void)
{
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x01);
  ceHigh();
  delayMicroseconds(130);
}
void nrf24::set_address_with(uint8_t aw)
{
      setRegister(_NRF24_SETUP_AW,aw);
}
uint8_t nrf24::getStatus()
{
	 *spi_buff = NOP;

  digitalWrite (csn_pin, LOW);
  wiringPiSPIDataRW (0, spi_buff, 1);
  digitalWrite (csn_pin, HIGH);

    return *spi_buff;
}

byte * nrf24::getAddress(byte * address_buf,byte typ,byte number_of_bytes)
{
  uint8_t * ptx = spi_buff;
  uint8_t * prx = spi_buff;

 *ptx = ( R_REGISTER | ( REGISTER_MASK & typ ) );
  digitalWrite (csn_pin, LOW);
  wiringPiSPIDataRW (0, spi_buff, number_of_bytes+1);
  digitalWrite (csn_pin, HIGH);
  prx++;
  for(int i=0; i<number_of_bytes; ++i) *address_buf++ = *prx++;
  return address_buf;
}

void nrf24::printConfig()
{
    printf("Config: %#04x\n",getRegister(_NRF24_CONFIG));
}
void nrf24::printStatus()
{
   // printf("Status: %#04x\n",getStatus());
	std::bitset<8> x(getStatus());
	std::cout << "Status: " << x << std::endl;
}


void nrf24::printRegister(uint8_t reg)
{
	std::bitset<8> x(getRegister(reg));
	std::cout << "Rigister " << getRegName(reg) << ": " << x << std::endl;
}

void nrf24::printAddresses()
{
	byte address[5];

	// Print RX_P0
	getAddress(address,_NRF24_RX_ADDR_P0,5);
	std::cout << "Address Rx P0: "<<std::hex;
	for(int i=0;i<5;++i) {
		 std::cout << static_cast<int>(address[i]);
	}
	std::cout << std::endl;

	// Print RX_P1
	getAddress(address,_NRF24_RX_ADDR_P1,5);
	std::cout << "Address Rx P1: "<<std::hex;
	for(int i=0;i<5;++i) {
		 std::cout << static_cast<int>(address[i]);
	}

	// Print RX_P2 to P5
	std::cout << std::endl;
	for(int i=0; i<4; ++i)
	{
	std::cout << "Address Rx P"<<i+2<<": "<<std::hex
			<< static_cast<int>(getRegister(_NRF24_RX_ADDR_P2+i)) << std::endl;
	}

	// Print TX
	getAddress(address,_NRF24_TX_ADDR,5);
	std::cout << "Address TX: "<<std::hex;
	for(int i=0;i<5;++i) {
		 std::cout << static_cast<int>(address[i]);
	}
	std::cout << std::endl;

}


