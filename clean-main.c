#include "stm32f10x.h"

#if defined(USE_STDPERIPH_DRIVER)
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#endif

void init_rcc(void) {
#if !defined(USE_STDPERIPH_DRIVER)
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
#else
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08);
	}
#endif
}
/*
  #define  USARTx                     USART1
  #define  GPIOx                      GPIOA
  #define  RCC_APB1Periph_USARTx      RCC_APB2Periph_USART1
  #define  GPIO_RxPin                 GPIO_Pin_10
  #define  GPIO_TxPin                 GPIO_Pin_9
*/
void init_usart1(void) {
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;     // 115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	//TODO: enable1 usart clock
	
	//TODO: configure usart1 gpios
	
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void init_adc(void) {
	// ADCCLK(max 14Mhz)
	// XXX: IFI overclocked the ADC to 18MHz.
	//  PCLK2 /6 = 12 + 2/3 MHz
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE)
		| RCC_CFG_ADCPRE_DIV6;
}

void init_gpio(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//TODO: enable gpio clock.
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinRemapConfig(
		  GPIO_Remap_USART2 
		| GPIO_PartialRemap2_TIM2
		| GPIO_PartialRemap_USART3
		| GPIO_FullRemap_TIM3
		, ENABLE);
}

void main(void)
{
#ifdef DEBUG
	debug();
#endif
	init_rcc();

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
