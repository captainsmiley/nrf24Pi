#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <stddef.h>
#include "bcm2835.h"

//SPI receive buffer (payload max 32 bytes)
uint8_t spi_rxbuff[32] ;
//SPI transmit buffer (payload max 32 bytes + 1 byte )
uint8_t spi_txbuff[32+1] ; 

// nRF24L01P registers we need
#define _NRF24_CONFIG      0x00
#define _NRF24_EN_AA       0x01
#define _NRF24_SETUP_AW    0x03
#define _NRF24_RF_CH       0x05
#define _NRF24_RF_SETUP    0x06
#define _NRF24_RPD         0x09
#define byte unsigned char
#define NOP           0xFF
#define R_REGISTER    0x00
#define REGISTER_MASK 0x1F
#define W_REGISTER    0x20


uint8_t ce_pin =RPI_V2_GPIO_P1_22 ; //25
uint8_t csn_pin =RPI_V2_GPIO_P1_24 ; //8

// get the value of a nRF24L01p register
byte getRegister(uint8_t reg)
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
void setRegister(uint8_t reg, uint8_t value)
{
  uint8_t status;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;

 *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
 *ptx = value ;
  bcm2835_gpio_write(csn_pin, LOW);	
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
  bcm2835_gpio_write(csn_pin,HIGH);	
  status = *prx++; // status is 1st byte of receive buffer
}

// power up the nRF24L01p chip
void powerUp(void)
{
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x02);
  delayMicroseconds(130);
}

// switch nRF24L01p off
void powerDown(void)
{
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)&~0x02);
}

// enable RX 
void ceHigh(void)
{
    bcm2835_gpio_write(ce_pin, HIGH); 
}

// disable RX
void ceLow(void)
{
    bcm2835_gpio_write(ce_pin, LOW);
}

// setup RX-Mode of nRF24L01p
void setRX(void)
{
  setRegister(_NRF24_CONFIG,getRegister(_NRF24_CONFIG)|0x01);
  ceHigh();
  delayMicroseconds(130);
}
void set_address_with(uint8_t aw)
{
      setRegister(_NRF24_SETUP_AW,aw);
}
uint8_t getStatus()
{
    *spi_rxbuff = NOP;

    bcm2835_gpio_write(csn_pin, LOW);	
    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 1);
    bcm2835_gpio_write(csn_pin,HIGH);	
    return *spi_txbuff; // status is 1st byte of receive buffer
}

byte * getAddress(byte * address_buf,byte typ,byte number_of_bytes)
{
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;

 *ptx = ( R_REGISTER | ( REGISTER_MASK & typ ) );
  bcm2835_gpio_write(csn_pin, LOW);	
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, number_of_bytes+1);
  bcm2835_gpio_write(csn_pin,HIGH);	
  prx++;
  for(int i=0; i<number_of_bytes; ++i) *address_buf++ = *prx++;
  return address_buf;
}

void printConfig()
{
    printf("%#04x\n",getRegister(_NRF24_CONFIG));
}
void printStatus()
{
    printf("%#04x\n",getRegister(_NRF24_CONFIG));
}




void setup()
{
   
  printf("Starting Tobias NRF24 [raspberry pi]\n");
  printf("\n");
  
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
  
  // now start receiver
  powerUp();
  
  // switch off Shockburst
  setRegister(_NRF24_EN_AA,0x0);
  
  // make sure RF-section is set properly 
  // - just write default value... 
  setRegister(_NRF24_RF_SETUP,0x0F); 


  
}

void loop() 
{ 

}


int main(int argc, char** argv)
{
    setup();
    while(1)
        loop();

    return 0;
}

