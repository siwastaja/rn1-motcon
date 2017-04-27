# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m0 -Wall -fstack-usage -Winline
ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m0 -mthumb -nostartfiles -gc-sections

DEPS = main.h own_std.h flash.h
OBJ = stm32init.o main.o own_std.o flash.o
ASMS = stm32init.s main.s own_std.s flash.s

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^
	$(OBJCOPY) -Obinary main.elf main_full.bin
	$(OBJCOPY) -Obinary --remove-section=.flasher --remove-section=.settings main.elf main.bin
	$(SIZE) main.elf

flash_full: main.bin
	sudo stm32sprog -d /dev/ttyUSB1 -b 230400 -vw main_full.bin

flash: main.bin
	sudo stm32sprog -d /dev/ttyUSB1 -b 230400 -vw main.bin

f: main.bin
	../rn1-tools/mcprog ~/dev/robo ./main.bin

f_local: main.bin
	sudo ../rn1-tools/mcprog /dev/ttyUSB0 ./main.bin 4
	sudo ../rn1-tools/mcprog /dev/ttyUSB0 ./main.bin 3

f4: main.bin
	sudo ../rn1-tools/mcprog /dev/ttyUSB0 ./main.bin 4

f3: main.bin
	sudo ../rn1-tools/mcprog /dev/ttyUSB0 ./main.bin 3

ff: main.bin
	scp main.bin hrst@proto4:~/mc.bin

stack:
	cat *.su

sections:
	arm-none-eabi-objdump -h main.elf

syms:
	arm-none-eabi-objdump -t main.elf

%.s: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e: 
	nano main.c stm32init.c stm32.ld motcon_comm.txt
