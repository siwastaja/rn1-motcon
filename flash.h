/*
	PULUROBOT RN1-MOTCON  Motor controller MCU firmware

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.

*/

#ifndef __FLASH_H
#define __FLASH_H

// Blocks until free space in the SPI TX FIFO
void spi1_poll_tx(uint16_t d);

// Blocks until data available in SPI RX FIFO - so indefinitely unless you have issued a TX just before.
uint16_t spi1_poll_rx();

// Empties the rx fifo
void spi1_empty_rx();

#endif
