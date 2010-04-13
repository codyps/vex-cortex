
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

	USART_InitStructure.USART_BaudRate = 115200;
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
	GPIOA->CRH |= GPIO_CRH_MODE9_1
		| GPIO_CRH_MODE9_0
		| GPIO_CRH_CNF9_1;
	
	// RX = PA10 = Floating input.
	// mode = 00, CNF = 01
	GPIOA->CRH &= ~(GPIO_CRH_MODE10
		| GPIO_CRH_CNF10);
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
	
	/* USART */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
/* Transmit
1. Enable the USART by writing
	the UE bit in USART_CR1 register to 1.
2. Program the M bit in USART_CR1 to
	define the word length.
3. Program the number of stop bits 
	in USART_CR2.
4. Select DMA enable (DMAT) in USART_CR3
	if Multi buffer Communication is 
	to take place. Configure the DMA 
	register as explained in multibuffer 
	communication.
5. Select the desired baud rate using 
	the USART_BRR register.
6. Set the TE bit in USART_CR1 to send
	an idle frame as first transmission.
7. Write the data to send in the USART_DR
	register (this clears the TXE bit). 
	Repeat this for each data to be 
	transmitted in case of single buffer.
8. After writing the last data into the
	USART_DR register, wait until TC=1. 
	This indicates that the transmission
	of the last frame is complete. This
	is required for instance when the
	USART is disabled or enters the Halt 
	mode to avoid corrupting the last 
	transmission.
*/

/* Recieve
1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
2. Program the M bit in USART_CR1 to define the word length.
3. Program the number of stop bits in USART_CR2.
4. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to take
place. Configure the DMA register as explained in multibuffer communication. STEP 3
5. Select the desired baud rate using the baud rate register USART_BRR
6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a
start bit.
*/
	
	// Enable USART
	USART1->CR1 |= USART_CR1_UE;
	
	// 8bit word len
	USART1->CR1 &= ~USART_CR1_M;
	
	// 1 stop bit = 0b00
	USART1->CR2 &= ~USART_CR2_STOP;

	// FIXME: set parity and flow control.

#define USART_BRR(_fclk_,_baud_)            \
	( (uint32_t)  (                     \
	  (uint32_t)                        \
		_fclk_ * 0xF /  _baud_ / 16 \
	)             )
	
	// Baud rate, DIV = fclk / (16*baud)
	// fclk = ABP2 clk = HCLK = 72Mhz = 
	// 115200               xx xx xx
	USART1->BRR = USART_BRR(72000000 ,115200);

	// enable reciever and transmiter.
	// TODO:	


	/** USART2: **/
	/* REMAP */
	// TX = PD5 , RX = PD6
	// CK = PD7, CTS = PD3 , RTS = PD4
	AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
	
	/* GPIO */
	// TODO:

	/* USART */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// TODO:

	/** USART3: **/
	/* REMAP */
	// TX = PC10, RX = PC11
	// CK = PC12, CTS = PB13, RTS = PB14
	AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
	AFIO->MAPR |= 
		AFIO_MAPR_REMAP_PARTIALREMAP;
		
	/* GPIO */
	// TODO:

	/* USART */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	// TODO:
	
}


void tim_init(void) {
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

void gpio_init(void) {
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
