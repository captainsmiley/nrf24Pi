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


void loop(nrf24 & r)
{

	delay(10);
	r.ceLow();
	if(r.getStatus() & 0b01000000)
	{
		byte a[32] = {};
		r.readFIFO(&a[1],1);
		std::cout << "Received: " << a[1] << std::endl;
		/*
		r.readFIFO(&a[1],1);
		std::cout << "Received: " << a[1] << std::endl;
		r.readFIFO(a,1);
		std::cout << "Received: " << a[1] << std::endl;
		 */
		r.clearInt_RX_DR();
		r.printStatus();
	}
	r.ceHigh();

}


int main(int argc, char** argv)
{
	std::cout << "Program starting" << std::endl;
	nrf24 r1;



	//setup();
	while(1)
		loop(r1);

	return 0;
}

