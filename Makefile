TARGET=main
SOURCE=clean-main.c
HEADER=$(wildcard *.h)
OBJ=$(SOURCE:=.o)

GCC_PATH=
GCC_PREFIX=$(GCC_PATH)arm-none-eabi-

CC=$(GCC_PREFIX)gcc
AS=$(GCC_PREFIX)gcc
LD=$(GCC_PREFIX)gcc
OBJCOPY=$(GCC_PREFIX)objcopy

INCLUDES=-I. -I./fwlib-3.2.0/inc
LD_INC=-L./lib

ALL_CFLAGS=-MD -D_STM32F103VDH6_ -D_STM3x_ -D_STM32x_ \
           -mthumb -mcpu=cortex-m3 -Wall -g \
		   $(INCLUDES) $(CFLAGS)
ALL_LDFLAGS=$(ALL_CFLAGS)\
            -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler \
			$(LD_INC) -T stm32.ld

all: $(TARGET).hex $(TARGET).bin

%.c.o: %.c $(HEADER)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

%.elf: $(OBJ)
	$(LD) $(ALL_LDFLAGS) -o $@ $^

%.hex: %.elf
	$(OBJCOPY) -S -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -S -O bin $< $@
	
clean:
	$(MAKE) -C lib clean
	$(MAKE) -C fwlib-3.2.0 clean
	$(RM) -rf *.o *.elf *.hex *.bin
