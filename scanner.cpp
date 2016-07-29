/*
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <stddef.h>
#include "bcm2835.h"
 */

#include <iostream>
#include "nrf24.h"
#include <wiringPi.h>





void loop(nrf24 & r)
{
	delay(1000);
	/*
	r.ceLow();
	if(r.getStatus() & 0b01000000)
	{
		byte a[32] = {};
		r.readFIFO(&a[1],1);
		std::cout << "Received: " << a[1] << std::endl;
		std::cout << "Received: " << a[1] << std::endl;
		r.readFIFO(a,1);
		std::cout << "Received: " << a[1] << std::endl;
		r.clearInt_RX_DR();
		r.printStatus();

	}
	r.ceHigh();
*/

		r.printStatus();
}


int main(int argc, char** argv)
{
	std::cout << "Program starting" << std::endl;

	wiringPiSetup();
	nrf24 r1;
	r1.setup(nrf24::RX);

	//pinMode (0, OUTPUT) ;
	pinMode (6, OUTPUT) ;
	//digitalWrite(0,HIGH);
	digitalWrite(6,HIGH);


	//wiringPiISR (0, INT_EDGE_BOTH,  &int_handler) ;


	//setup();
	while(1) ;
		loop(r1);

	return 0;
}

