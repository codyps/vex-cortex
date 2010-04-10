/* From system_stm32f10x.c, 
 * STD PERIPH LIB 3.2.0
 */

#include <stm32f10x.h>
#include <stm32f10x_rcc.h>

static void rcc_reset(void)
{
	/* Reset the RCC clock
	 * configuration to the 
	 * default reset state 
	 */
	/* Set HSION bit */
	RCC->CR |= RCC_CR_HSION;

	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
#if !defined(STM32F10X_CL)
	RCC->CFGR &= (uint32_t)0xF8FF0000;
#else
	RCC->CFGR &= (uint32_t)0xF0FF0000;
#endif /* STM32F10X_CL */   

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= ~(RCC_CR_HSEON 
		| RCC_CR_CSSON 
		| RCC_CR_PLLON);

	/* Reset HSEBYP bit */
	RCC->CR &= ~(RCC_CR_HSEBYP);

	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
	RCC->CFGR &= (uint32_t)0xFF80FFFF;
	/*
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC
		| RCC_CFGR_PLLXTPRE
		| RCC_CFGR_PLLMUL);
	*/

#if defined( STM32F10X_CL)
	/* Reset PLL2ON and PLL3ON bits */
	RCC->CR &= ~(RCC_CR_PLL2ON
		| RCC_CR_PLL3ON);
	/* Disable all interrupts and clear pending bits  */
	RCC->CIR = 0x00FF0000;

	/* Reset CFGR2 register */
	RCC->CFGR2 = 0x00000000;
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) 
	/* Disable all interrupts and clear pending bits  */
	RCC->CIR = 0x009F0000;

	/* Reset CFGR2 register */
	RCC->CFGR2 = 0x00000000;      
#else
	/* Disable all interrupts and clear pending bits  */
	RCC->CIR = 0x009F0000;
#endif /* STM32F10X_CL */

}

static void rcc_setup(void)
{	
	/** SYSCLK, HCLK, PCLK2 and PCLK1 configuration **/    
	/* Enable HSE */    
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready and if Time out is reached exit */
	{
		uint32_t i = 0;
		do {
			i++;  
		} while (
			( (RCC->CR & RCC_CR_HSERDY) == 0 )
			 && (i != HSEStartUp_TimeOut));
	}
		
	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
		/* Enable Prefetch Buffer */
		FLASH->ACR |= FLASH_ACR_PRFTBE;

		/* Flash 2 wait state */
		FLASH->ACR &= ~FLASH_ACR_LATENCY;
		FLASH->ACR |= FLASH_ACR_LATENCY_2;    

		/* HCLK = SYSCLK */
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
		  
		/* PCLK2 = HCLK */
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

		/* PCLK1 = HCLK/2 */
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

#if defined(STM32F10X_CL)
		/** Configure PLLs **/
		/* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
		/* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
			
		RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV2
			| RCC_CFGR2_PLL2MUL
			| RCC_CFGR2_PREDIV1
			| RCC_CFGR2_PREDIV1SRC);
		RCC->CFGR2 |= RCC_CFGR2_PREDIV2_DIV5
			| RCC_CFGR2_PLL2MUL8
			| RCC_CFGR2_PREDIV1SRC_PLL2
			| RCC_CFGR2_PREDIV1_DIV5;

		/* Enable PLL2 */
		RCC->CR |= RCC_CR_PLL2ON;
		/* Wait till PLL2 is ready */
		while(!(RCC->CR & RCC_CR_PLL2RDY));

		/* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */ 
		RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE 
			| RCC_CFGR_PLLSRC
			| RCC_CFGR_PLLMULL);
		RCC->CFGR |= RCC_CFGR_PLLXTPRE_PREDIV1
			| RCC_CFGR_PLLSRC_PREDIV1
			| RCC_CFGR_PLLMULL9; 
#else    
		/*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
		RCC->CFGR &= ~(RCC_CFGR_PLLSRC 
			| RCC_CFGR_PLLXTPRE
			| RCC_CFGR_PLLMULL);
		RCC->CFGR |= RCC_CFGR_PLLSRC_HSE
			| RCC_CFGR_PLLMULL9;
#endif /* STM32F10X_CL */

		/* Enable PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till PLL is ready */
		while(!(RCC->CR & RCC_CR_PLLRDY));

		/* Select PLL as system clock source */
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_PLL;    

		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR & RCC_CFGR_SWS)
			!= (uint32_t)0x08);
		
	} else { 
		/* If HSE fails to start-up, the application will 
			have wrong clock configuration. User can add
			here some code to deal with this error */
	}
}

static void rcc_setup(void) {
	// Assumptions on entry:
	//  running on HSI, PLL disabled.
	//  all periferal clocks disabled.

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

static void rcc_setup(void) 
{
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

	/* Flash 2 wait state */
	FLASH_SetLatency(FLASH_Latency_2);
	
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div1); 

	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1); 

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div2);

	if(HSEStartUpStatus == SUCCESS) {
		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	
	} else {
		// Apparently the HSE failed.
		// Lets attempt to use the HSI
		/* PLLCLK = 8MHz / 2 * 9 = 36 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
	}
	
	/* Enable PLL */ 
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while(RCC_GetSYSCLKSource() != 0x08);
}

//
void rcc_init(void) {
	rcc_reset();
	rcc_setup();
}
