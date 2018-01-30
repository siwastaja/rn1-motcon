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


	Routines for remote firmware update through SPI.

*/

#include "ext_include/stm32f0xx.h"
#include "flash.h"

/*
settings_type settings __attribute__((section(".settings"))) =
{
};

extern unsigned int _SETTINGS_BEGIN;
extern unsigned int _SETTINGS_END;
extern unsigned int _SETTINGSI_BEGIN;
*/

#define FLASH_OFFSET 0x08000000
#define LED_ON()  {GPIOF->BSRR = 1UL;}
#define LED_OFF() {GPIOF->BSRR = 1UL<<16;}

// Blocks until free space in the SPI TX FIFO
void spi1_poll_tx(uint16_t d) __attribute__((section(".flasher")));
void spi1_poll_tx(uint16_t d)
{
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = d;
}

// Blocks until data available in SPI RX FIFO - so indefinitely unless you have issued a TX just before.
uint16_t spi1_poll_rx() __attribute__((section(".flasher")));
uint16_t spi1_poll_rx()
{
	while(!(SPI1->SR & (1UL<<0))) ;
	return SPI1->DR;
}

// Empties the rx fifo
void spi1_empty_rx() __attribute__((section(".flasher")));
void spi1_empty_rx()
{
	while(SPI1->SR&(0b11<<9)) SPI1->DR;
}

extern void delay_ms(uint32_t i) __attribute__((section(".flasher")));
extern void delay_us(uint32_t i) __attribute__((section(".flasher")));

void unlock_flash() __attribute__((section(".flasher")));
void unlock_flash()
{
	if(FLASH->CR & (1UL<<7))
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

void lock_flash() __attribute__((section(".flasher")));
void lock_flash()
{
	FLASH->CR |= 1UL<<7;
}

void erase_page(uint32_t addr) __attribute__((section(".flasher")));
void erase_page(uint32_t addr)
{
	while(FLASH->SR & 1) ; // Poll busy bit
	FLASH->CR |= 1UL<<1; // Page erase
	FLASH->AR = addr;
	FLASH->CR |= 1UL<<6; // Start
	__asm__ __volatile__ ("nop");
	while(FLASH->SR & 1) ; // Poll busy bit
	while(!(FLASH->SR & (1UL<<5))) ; // Poll End Of Operation bit
	FLASH->SR |= 1UL<<5; // Clear End Of Operation bit by writing '1'

	__asm__ __volatile__ ("nop");
	while(FLASH->SR & 1) ; // Poll busy bit

	// STM32 documentation workaround: PAGE ERASE BIT MUST BE CLEARED!!! This is undocumented.
	// Writing PG bit fails without any warning or error flags if PER is not cleared.
	FLASH->CR = 0; // Erase bit off.
}

void spi_flash_program(int size) __attribute__((section(".flasher")));
void spi_flash_program(int size)
{
	int i;

	FLASH->CR = 1UL; // Activate programming

	uint16_t* p_flash = (uint16_t*)FLASH_OFFSET;

	for(i = 0; i < size; i++)
	{
		*p_flash = spi1_poll_rx();
		spi1_poll_tx(0x1111);
		p_flash++;
		while(FLASH->SR & 1) ; // Poll busy bit
	}

	FLASH->SR |= 1UL<<5; // Clear End Of Operation bit by writing '1'
	FLASH->CR = 0; // Clear programming bit.
}

void spi_flash_read(int size) __attribute__((section(".flasher")));
void spi_flash_read(int size)
{
	int i;

	uint16_t* p_flash = (uint16_t*)FLASH_OFFSET;

	for(i = 0; i < size; i++)
	{
		spi1_poll_tx(*p_flash); // blocks to sync to the rn1-brain uart speed
		spi1_empty_rx();
		p_flash++;
	}
}

/*
Commands (In MSByte of the 16-bit SPI word)

100: Erase pages (LSByte defines how many 1K pages are erased)

Send dummy data until you receive 0xaaaa (erase done)


101: Write (next data: number of 16-bit words)

No response; just write the words. I'm not sure about the max speed allowed, but if the programmer side is
limited by 115200 uart, that's ok at least.


102: Read (next data: number of 16-bit words)

Give clock: data is returned (no status codes)

*/

void flasher() __attribute__((section(".flasher")));
void flasher()
{
	int i;
	uint16_t size;

	LED_ON();

	while(1)
	{
		uint16_t datareg = spi1_poll_rx();
		uint8_t cmd = (datareg&0xff00)>>8;
		uint8_t arg = datareg&0xff;

		switch(cmd)
		{
			case 100:
			if(arg < 1 || arg > 30)
			{
				{while(1) { delay_ms(50); LED_ON(); delay_ms(50); LED_OFF(); }}			
			}
			unlock_flash();
			for(i=0; i < arg; i++)
			{
				erase_page(FLASH_OFFSET + i*1024);
			}

			lock_flash();
			spi1_poll_tx(0xaaaa);
			spi1_empty_rx();
			break;

			case 101:
			spi1_poll_tx(0xcccc);
			size = spi1_poll_rx();
			if(size < 50 || size > 30*1024)
			{
				{while(1) { delay_ms(50); LED_ON(); delay_ms(50); LED_OFF(); }}			
			}
			unlock_flash();
			spi_flash_program(size);
			spi1_empty_rx();
			lock_flash();
			break;

			case 102:
			spi1_poll_tx(0xcccc); // Master can recognize this and knows the data will follow.
			size = spi1_poll_rx();
			if(size < 50 || size > 30*1024)
			{
				{while(1) { delay_ms(50); LED_ON(); delay_ms(50); LED_OFF(); }}			
			}
			spi_flash_read(size);
			spi1_empty_rx();
			break;

			case 150:
			case 151:
			LED_OFF();
			NVIC_SystemReset();
			while(1);

			default:
			break;
		}

	}
}
