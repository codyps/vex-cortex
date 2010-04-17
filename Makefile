TARGET = main
FWLIB_DIR = $(srcdir)/lib/fwlib
STM_LIB_SRC= $(srcdir)/lib/startup/startup_stm32f10x_hd.s \
             $(wildcard $(FWLIB_DIR)/src/*.c)

SOURCE=main.c     \
       rcc.c      \
       usart.c    \
       stm32f10x_it.c \
       $(STM_LIB_SRC)

srcdir=.
#VPATH=$(srcdir)/lib/fwlib/src $(srcdir)

HEADER=$(wildcard *.h)
OBJ=$(SOURCE:=.o)

GCC_PATH=
ARCH_PREFIX=$(GCC_PATH)arm-none-eabi-

FIND=find
CC=$(ARCH_PREFIX)gcc
AS=$(ARCH_PREFIX)gcc
LD=$(ARCH_PREFIX)gcc
NM=$(ARCH_PREFIX)nm
OBJDUMP=$(ARCH_PREFIX)objdump
OBJCOPY=$(ARCH_PREFIX)objcopy
STRIP=$(ARCH_PREFIX)strip

CC_INC=-I$(srcdir) -I$(srcdir)/lib/fwlib/inc -I$(srcdir)/lib
LD_INC=-L$(srcdir)/lib -L$(srcdir)/ld

# When changing boards, modify STMPROC,
# HSE_VALUE, and the startup asm code.
# And the linker script
LD_SCRIPT=stm32f103VD.ld
STMPROC=STM32F10X_HD
HSE_VALUE=8000000

ALL_CFLAGS=-MD -D$(STMPROC) -DHSE_VALUE=$(HSE_VALUE) \
           -mthumb -mcpu=cortex-m3 -Wall             \
           -Wno-main -DUSE_STDPERIPH_DRIVER -pipe    \
           -ffunction-sections                       \
	   $(CC_INC) $(CFLAGS)
ALL_LDFLAGS=$(ALL_CFLAGS)                            \
	    -nostartfiles                            \
            -Wl,--gc-sections,-Map=$@.map,-cref      \
            -Wl,-u,Reset_Handler                     \
	    -fwhole-program -Wl,-static              \
            $(LD_INC) -T $(LD_SCRIPT)

ALL_ASFLAGS=$(ALL_CFLAGS)

.SUFFIXES:

all: $(TARGET).hex $(TARGET).bin $(TARGET).elf.strip \
	$(TARGET).elf.sym  $(TARGET).elf.lss

Makefile:;

.SECONDARY:

%.s.o: %.s $(HEADER)
	$(AS) $(ALL_ASFLAGS) -c -o $@ $<

%.c.o: %.c $(HEADER)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

%.elf: $(OBJ)
	$(LD) $(ALL_LDFLAGS) -o $@ $^

%.hex: %.elf
	$(OBJCOPY) -S -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

# Create extended listing file from ELF output file.
%.elf.lss: %.elf
	$(OBJDUMP) -h -S $< > $@
# Create a symbol table from ELF output file.
%.elf.sym: %.elf
	$(NM) -n $< > $@

%.elf.strip: %.elf
	$(STRIP) -S $< -o $@
	$(NM) -n $<.strip > $@.sym
	$(OBJDUMP) -h -S $<.strip > $@.lss


clean:
	@$(FIND) . -regex '.*\.\([od]\|elf\|hex\|bin\|map\|lss\|sym\|strip\)'\
		-printf 'RM %P\n' -delete

.PHONY: clean all
