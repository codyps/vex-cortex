TARGET=main
SOURCE=main.c     \
       rcc.c      \
       $(STM_LIBC)

#srcdir=.
#VPATH=$(srcdir)/lib/fwlib/src $(srcdir)

HEADER=$(wildcard *.h)
OBJ=$(SOURCE:=.o)

GCC_PATH=
GCC_PREFIX=$(GCC_PATH)arm-none-eabi-

FIND=find
CC=$(GCC_PREFIX)gcc
AS=$(GCC_PREFIX)gcc
LD=$(GCC_PREFIX)gcc
OBJCOPY=$(GCC_PREFIX)objcopy

INCLUDES=-I. -I./lib/fwlib/inc -I./lib
LD_INC=-L./lib -L./ld

STMPROC=STM32F10X_HD
HSE_VALUE=((uint32_t)8000000)
STM_LIBC= ./lib/fwlib/src/stm32f10x_gpio.c\
          ./lib/fwlib/src/stm32f10x_usart.c

ALL_CFLAGS=-MD -D$(STMPROC) -DHSE_VALUE="$(HSE_VALUE)" \
           -mthumb -mcpu=cortex-m3 -Wall -g \
            $(INCLUDES) $(CFLAGS)
ALL_LDFLAGS=$(ALL_CFLAGS)\
            -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler \
            $(LD_INC) -T STM32F103VD.ld

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
	@$(FIND) . -regex '.*\.\([od]\|elf\|hex\|bin\)' -printf 'RM %P\n' -delete

