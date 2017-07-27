/* vim:fdm=marker ts=4 et ai
 * {{{
 *
 * (c) by Alexander Neumann <alexander@bumpern.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 }}} */

#include <avr/io.h>
#include "spi.h"

void spi_init(void)
{
    //DDRB  |= 1<<PB5 | 1<<PB7; // mosi, sck output
	DDRB  |= 1<<SPI_MOSI | 1<<SPI_SCK; // mosi, sck output
	cbi(SPI_DDR,SPI_MISO); // MISO is input
        
           //    CSPASSIVE; ###############################
    cbi(SPI_PORT,SPI_MOSI); // MOSI low
    cbi(SPI_PORT,SPI_SCK); // SCK low
	//

    //CPOL = 0;
    //CPHA = 0;
  //  SPCR &= 0xF3;

	// initialize SPI interface
	// master mode and Fosc/2 clock:
    SPCR = (1<<SPE)|(1<<MSTR);  
    SPCR &= 0xFC; // SPR1=0, SPR0 = 0  this is ori
     //SPCR &= 0x7B;
   // master mode and Fosc/128 clock:
   //SPCR &= 0xFB;
  
	SPSR = 0x00;
   // SPSR |= (1<<SPI2X); 

} 

