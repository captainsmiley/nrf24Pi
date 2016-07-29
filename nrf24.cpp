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

	// Setup GPIO
	  bcm2835_init();

	  // Activate Chip Enable
	  bcm2835_gpio_fsel(ce_pin, BCM2835_GPIO_FSEL_OUTP);
	  bcm2835_gpio_fsel(csn_pin, BCM2835_GPIO_FSEL_OUTP);
	  bcm2835_gpio_write(ce_pin, LOW);
	  bcm2835_gpio_write(csn_pin, HIGH);

	  // Setup SPI
	  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

	  // Set SPI bus Speed
	  bcm2835_spi_setClockSpeed(BCM2835_SPI_SPEED_1MHZ);
	  // This initialize the SPI bus with
	  // csn pin as chip select (custom or not)
	  bcm2835_spi_begin(csn_pin);
	  delay(100);
	  ceLow();

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
	printRegister(_NRF24_FIFO_STATUS);
	printAddresses();

	ceHigh();



}

nrf24::~nrf24() {
	// TODO Auto-generated destructor stub
}

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
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
	*ptx++ = ( R_RX_PAYLOAD );
	for(int i=0; i<nr_bytes;++i) *ptx++ = NOP ; // Dummy operation
	bcm2835_gpio_write(csn_pin, LOW);
	bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, nr_bytes+1);
	bcm2835_gpio_write(csn_pin,HIGH);
	for(int i=0; i<nr_bytes; ++i)
	{
		*(buff+i) = *(prx+1+i);
	}

}

void nrf24::clearInt_RX_DR()
{
	setRegister(_NRF24_STATUS,(getStatus() | 0b01000000 ));
}


// get the value of a nRF24L01p register
byte nrf24::getRegister(uint8_t reg)
{
uint8_t result;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
  *ptx = NOP ; // Dummy operation, just for reading
  bcm2835_gpio_write(csn_pin, LOW);
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
  bcm2835_gpio_write(csn_pin,HIGH);
  result = *++prx;   // result is 2nd byte of receive buffer
  return (result);
}

// set the value of a nRF24L01p register
void nrf24::setRegister(uint8_t reg, uint8_t value)
{
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;

 *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
 *ptx = value ;
  bcm2835_gpio_write(csn_pin, LOW);
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
  bcm2835_gpio_write(csn_pin,HIGH);
}
// power up the nRF24L01p chip
void nrf24::powerUp(void)
{std::cout << "Running nrf24 powerUp" << std::endl;
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x02);
  delayMicroseconds(130);
std::cout << "Running nrf24 powerUp end" << std::endl;
}

// switch nRF24L01p off
void nrf24::powerDown(void)
{
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)&~0x02);
}

// enable RX
void nrf24::ceHigh(void)
{
    bcm2835_gpio_write(ce_pin, HIGH);
}

// disable RX
void nrf24::ceLow(void)
{
    bcm2835_gpio_write(ce_pin, LOW);
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
	 *spi_txbuff = NOP;

	  bcm2835_gpio_write(csn_pin, LOW);
	  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 1);
	  bcm2835_gpio_write(csn_pin,HIGH);

    return *spi_rxbuff;
}

byte * nrf24::getAddress(byte * address_buf,byte typ,byte number_of_bytes)
{
  uint8_t * ptx = spi_txbuff;
  uint8_t * prx = spi_rxbuff;

 *ptx = ( R_REGISTER | ( REGISTER_MASK & typ ) );
  bcm2835_gpio_write(csn_pin, LOW);
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, number_of_bytes+1);
  bcm2835_gpio_write(csn_pin,HIGH);
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


