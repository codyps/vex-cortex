#define USE_STDPERIPH_DRIVER

#include "stm32f10x.h"

#if defined(USE_STDPERIPH_DRIVER)
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#endif


#include "rcc.h"

/*
  #define  USARTx                     USART1
  #define  GPIOx                      GPIOA
  #define  RCC_APB1Periph_USARTx      RCC_APB2Periph_USART1
  #define  GPIO_RxPin                 GPIO_Pin_10
  #define  GPIO_TxPin                 GPIO_Pin_9
*/
void usart1_init(void) {
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

void adc_init(void) {
	// ADCCLK(max 14Mhz)
	// XXX: IFI overclocked the ADC to 18MHz.
	//  PCLK2 /6 = 12 + 2/3 MHz
	//RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE)
	//	| RCC_CFG_ADCPRE_DIV6;
}

void gpio_init(void) {
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
	rcc_init();

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
