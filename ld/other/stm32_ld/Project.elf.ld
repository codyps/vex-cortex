
/*
!!! Primary linker script file generated automatically !!!
This file contains the input and output files names
It also includes the secondary/main linker script that is specified in the Ride options (default or custom)
*/

SEARCH_DIR(".")
SEARCH_DIR("C:\Users\Nyx\Downloads\Vex2_default\Source\obj")
SEARCH_DIR("C:\Program Files\Raisonance\Ride\Lib\ARM")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_vector.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\cortexm3_macro.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\utilities.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\ddt_support.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\main.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_it.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_adc.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_dbgmcu.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_dma.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_exti.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_flash.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_gpio.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_i2c.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_iwdg.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_lib.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_nvic.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_pwr.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_rcc.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_rtc.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_sdio.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_spi.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_systick.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_tim.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_usart.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_bkp.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_can.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_crc.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_dac.o")
INPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\stm32f10x_wwdg.o")
EXTERN( __io_putchar )
EXTERN( _write )
GROUP(
"STM32x_io_putchar_thumb.a"
"std_sbrk_thumb.lib"
"smallprintf_thumb.a"
"libc.a"
"libm.a"

)
INCLUDE "C:\Users\Nyx\Downloads\Vex2_default\Source\obj\STM32F103_384K_64K_FLASH.ld"
OUTPUT("C:\Users\Nyx\Downloads\Vex2_default\Source\obj\Project.elf")
