
#include <string.h>
#include <stdio.h>

#include "stm32f10x.h"
#include "vex_hw.h"


#if defined(USE_STDPERIPH_DRIVER)
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#endif

#include "compilers.h"
#include "rcc.h"
#include "usart.h"
#include "spi.h"

#if 0
static void adc_init(void)
{
	// ADCCLK(max 14Mhz)
	// XXX: IFI overclocked the ADC to 18MHz.
	//  PCLK2 /6 = 12 + 2/3 MHz
	//RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE)
	//	| RCC_CFG_ADCPRE_DIV6;
}

static void tim_init(void)
{
	/** TIM2: **/
	// ETR/CH1 = PA0 , CH2 = PA1
	//     CH3 = PB10, CH4 = PB11
	AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;
	AFIO->MAPR |= 
		AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2;
	
	/** TIM3: **/
	// CH1 = PC6, CH2 = PC7
	// CH3 = PC8, CH4 = PC9
	AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP;
	AFIO->MAPR |=
		AFIO_MAPR_TIM3_REMAP_FULLREMAP;

}
#endif

static void gpio_init(void)
{
	// enable gpio clock (+ AFIO for good measure)
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN
		| RCC_APB2ENR_IOPAEN
		| RCC_APB2ENR_IOPBEN
		| RCC_APB2ENR_IOPCEN
		| RCC_APB2ENR_IOPDEN
		| RCC_APB2ENR_IOPEEN
		| RCC_APB2ENR_IOPFEN
		| RCC_APB2ENR_IOPGEN;

	// Set all gpios to "analog", aka diabled
	GPIOA->CRL = GPIOB->CRL 
		= GPIOC->CRL = GPIOD->CRL
		= GPIOE->CRL = GPIOF->CRL
		= GPIOG->CRL = 0;
	GPIOA->CRH = GPIOB->CRH
		= GPIOC->CRH = GPIOD->CRH
		= GPIOE->CRH = GPIOF->CRH
		= GPIOG->CRH = 0;

}

__noreturn void main(void)
{
	rcc_init();
	gpio_init();
	usart_init();
	spi_init();

	spi_packet_vex m2u, u2m;

	memset(&u2m,0,sizeof(u2m));
	memset(&m2u,0,sizeof(m2u));

	vex_spi_packet_init_u2m(&u2m);
	vex_spi_packet_init_m2u(&m2u);
	
	while(!is_master_ready()) {
		usart1_puts("** MASTER WAIT **\n");
	}
	
	usart1_puts("[ INIT DONE ]\n");
	
	for(;;) {
		if (transfer_flag) {
			vex_spi_xfer(&m2u,&u2m);
				
			usart1_puts("0123456789\n");
			//printf("hello\n");
		}
	}
}


#ifdef  USE_FULL_ASSERT
void assert_failed(u8* file, u32 line)
{ 
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	for(;;){
	}
}
#endif
