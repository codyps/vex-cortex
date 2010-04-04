#include "stm32f10x.h"

void init_RCC(void) {
	// Assumptions on entry:
	//  running on HSI, PLL disabled.

	// enable HSE
	RCC->CR |= RCC_CR_HSION;
	// wait for HSE startup (fwlib times out after 0x500 reads).
	while(!( RCC->CR & RCC_CR_HSIRDY));
	
	// FLASH: Enable Prefetch Buffer 
	//  (note: should only be done when running off of 8Mhz HSI.)
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	
    // FLASH: Set latency to 2
	FLASH->ACR |= FLASH_ACR_LATENCY_1;
	FLASH->ACR &= ~( FLASH_ACR_LATENCY_0
		| FLASH_ACR_LATENCY_2 );
	
    // HCLK: AHB (Max 72Mhz) 
	//  = SYSCLK = 72Mhz
	// 0xxx: SYSCLK not divided
	RCC->CFGR &= ~RCC_CFGR_HPRE_3;
	
	// PCLK2: APB2 (APB high speed, Max 72Mhz)
	//	= HCLK = 72MHz
	// 0xx: HCLK not divided
    RCC->CFGR &= ~RCC_CFGR_PPRE2_2;
	
    // PCLK1: APB1 (APB low speed, Max 36Mhz)
	//	= HCLK/2 = 36MHz
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1)
		| RCC_CFG_PPRE1_DIV2;
		
    // ADCCLK(max 14Mhz)
	// XXX: IFI overclocked the ADC to 18MHz.
	//  PCLK2 /6 = 12 + 2/3 MHz
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE)
		| RCC_CFG_ADCPRE_DIV6;
	
	// PLLCLK = 8MHz * 9 = 72 MHz
	// FIXME: not sure this is all the bits that need
	//		mucking.
	RCC->CFGR |= RCC_CFGR_PLLSRC;
	RCC->CFGR2 = (RCC->CFGR2 & ~RCC_CFGR2_PREDIV1)
		| RCC_CFGR2_PREDIV1_DIV1;
	RCC->CFGR  = (RCC->CFGR & ~RCC_CFGR_PLLMULL)
		| RCC_CFGR_PLLMULL9;
    
    // Enable PLL 
    RCC->CR |= RCC_CR_PLLON;

    // Wait till PLL is ready (no timeout here...)
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // Select PLL as system clock source
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW)
		| RCC_CFGR_SW_PLL;
	
    // Wait untill PLL is used as system clock source
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}

void main(void)
{
#ifdef DEBUG
	debug();
#endif
	init_RCC();

	for(;;) {
	}
}

#ifdef  DEBUG
void assert_failed(u8* file, u32 line)
{ 
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	for(;;){
	}
}
#endif
