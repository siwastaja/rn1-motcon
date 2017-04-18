#ifndef __FLASH_H
#define __FLASH_H

/***

This file defines all data that is stored in a specific flash page reserved for user-adjustable settings.

Data is stored in one struct, which is instructed to go into its own section. Linker script locates this
section in a separate flash page, but relocates it (using startup code in stm32init.c) in RAM normally,
like any other globals.

Hence, the data can be accessed just normally. Page can be cleared and rewritten, and new settings are
fetched on the next boot.
***/

typedef struct // __attribute__((packed))
{

} settings_type;

extern settings_type settings __attribute__((section(".settings")));

void save_settings();

#endif
