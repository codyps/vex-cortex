
#include "stm32f10x.h"

#if defined(USE_STDPERIPH_DRIVER)
#include "stm32f10x_rcc.h"
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
void usart1_init(void)
{
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

void adc_init(void)
{
	// ADCCLK(max 14Mhz)
	// XXX: IFI overclocked the ADC to 18MHz.
	//  PCLK2 /6 = 12 + 2/3 MHz
	//RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE)
	//	| RCC_CFG_ADCPRE_DIV6;
}

void usart_init(void) {
	/** USART1:
	TX = PA9, RX = PA10
	**/
	/* GPIO: */
	// TX = PA9 = AF push/pull =
	//  mode = 11, CNF = 10.
	GPIOA->CRH &= ~(GPIO_CRH_MODE9
		| GPIO_CRH_CNF9);
	GPIOA->CRH |= GPIO_CRH_MODE9_0
		| GPIO_CRH_MODE9_1
		| GPIO_CRH_CNF9_1;
	
	// RX = PA10 = Floating input.
	// mode = 00, CNF = 01
	GPIOA->CRH &= ~(GPIO_CRH_MODE10
		| GPIO_CRH_CNF10);
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
	
	/* USART */
	
	
	/** USART2: **/
	/* REMAP */
	// TX = PD5 , RX = PD6
	// CK = PD7, CTS = PD3 , RTS = PD4
	AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
	
	/* GPIO */
	
	/* USART */
	
	/** USART3: **/
	/* REMAP */
	// TX = PC10, RX = PC11
	// CK = PC12, CTS = PB13, RTS = PB14
	AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
	AFIO->MAPR |= 
		AFIO_MAPR_REMAP_PARTIALREMAP;
		
	/* GPIO */
	
	/* USART */
}

void gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// enable gpio clock.
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
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

__attribute__((noreturn)) void main(void)
{
	rcc_init();
	gpio_init();

	for(;;) {
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
