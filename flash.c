#include "ext_include/stm32f0xx.h"
#include "flash.h"

/*
	This file locates a settings data structure into a specific flash page, used for nonvolatile setting storage.
	This file also defines routines to erase, write and verify the flash memory for in-system SW upgrade through
	the SPI link. These routines are stored in yet another flash page, not erasable with these routines.
*/

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
void spi1_poll_tx(uint16_t d)
{
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = d;
}

// Blocks until data available in SPI RX FIFO - so indefinitely unless you have issued a TX just before.
uint16_t spi1_poll_rx()
{
	while(!(SPI1->SR & (1UL<<0))) ;
	return SPI1->DR;
}

// Empties the rx fifo
void spi1_empty_rx()
{
	while(SPI1->SR&(0b11<<9)) SPI1->DR;
}

extern void delay_ms(uint32_t i) __attribute__((section(".flasher")));;
extern void delay_us(uint32_t i) __attribute__((section(".flasher")));;

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

/*

void program_setting_page()
{
	unlock_flash();

	erase_page(FLASH_OFFSET + (uint32_t)&_SETTINGSI_BEGIN);

	FLASH->CR = 1UL<<0; // Program on.

	uint16_t* settings_begin  = (uint16_t*)&_SETTINGS_BEGIN;
	uint16_t* settings_end    = (uint16_t*)&_SETTINGS_END;
	volatile uint16_t* settingsi_begin = (uint16_t*)&_SETTINGSI_BEGIN;

	// When accessing flash, we don't use the remapped 0x0000 0000 address space, but the
	// real address space at FLASH_OFFSET. These being pointers to uint16_t (not bytes), we increment
	// the address by FLASH_OFFSET/2.
	settingsi_begin += FLASH_OFFSET/2;
	while(settings_begin < settings_end)
	{
		*settingsi_begin = *settings_begin;

		settings_begin++;
		settingsi_begin++;
		while(FLASH->SR & 1) ; // Poll busy bit

		// STM32 HW bug workaround: contrary to what the documentation clearly instructs you must do,
		// EOP bit must NOT be polled. It's not polled in official libraries, either.

	}

	FLASH->SR |= 1UL<<5; // Clear End Of Operation bit by writing '1'
	FLASH->CR = 0; // Program off.

	lock_flash();
}

int verify_settings()
{
	uint32_t* settings_begin  = (uint32_t*)&_SETTINGS_BEGIN;
	uint32_t* settings_end    = (uint32_t*)&_SETTINGS_END;
	uint32_t* settingsi_begin = (uint32_t*)&_SETTINGSI_BEGIN;

	while(settings_begin < settings_end)
	{
		if(*settings_begin != *settingsi_begin)
		{
			return -1;
		}
		settings_begin++;
		settingsi_begin++;
	}
	return 0;
}

void save_settings()
{
	program_setting_page();
	if(verify_settings())
	{
		program_setting_page();
		if(verify_settings())
		{
			error(FLASH_WRITE_ERROR);
		}
	}

}
*/

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

	LED_OFF();

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
				LED_ON(); while(1);
			}
			unlock_flash();
			for(i=0; i < arg; i++)
			{
				erase_page(FLASH_OFFSET + i*1024);	
			}

			lock_flash();
			spi1_poll_tx(0xaaaa);
			spi_empty_rx();
			break;

			case 101:
			spi1_poll_tx(0xcccc);
			size = spi1_poll_rx();
			if(size < 50 || size > 30*1024)
			{
				LED_ON(); while(1);
			}
			unlock_flash();
			spi_flash_program(size);
			spi_empty_rx();
			lock_flash();
			break;

			case 102:
			spi1_poll_tx(0xcccc); // Master can recognize this and knows the data will follow.
			size = spi1_poll_rx();
			if(size < 50 || size > 30*1024)
			{
				LED_ON(); while(1);
			}
			spi_flash_read(size);
			spi_empty_rx();
			break;

			case 150:
			case 151:
			NVIC_SystemReset();
			while(1);

			default:
			break;
		}

	}
}
