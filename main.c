
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "stm32f10x.h"
#include "vex_hw.h"


#if defined(USE_STDPERIPH_DRIVER)
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_misc.h"
#include "core_cm3.h"
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

static void nvic_init(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
}

static void tim1_init(void)
{
	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_TIM1
		, ENABLE);
	
	NVIC_InitTypeDef NVIC_param;	
	/* Enable the TIM1 global Interrupt */
	NVIC_param.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_param.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_param.NVIC_IRQChannelSubPriority = 3;
	NVIC_param.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_param);

	/* ---------------------------------------------------------------
	TIM1 Configuration: Output Compare Toggle Mode:
	TIM2CLK = 72 MHz, Prescaler = 20, 0xFFFF = 4.5ms
	--------------------------------------------------------------- */
	TIM_TimeBaseInitTypeDef  TIM_param;
	/* Time base configuration */
	TIM_param.TIM_Period = 65535;
	TIM_param.TIM_Prescaler = 20;
	TIM_param.TIM_ClockDivision = 0;
	TIM_param.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_param);

	/* TIM enable counter */
	TIM_Cmd(TIM1, ENABLE);

	/* TIM IT enable */
	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
	TIM1->SMCR &= 0xFFF8;
	
}

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

volatile bool spi_transfer_flag = true;

__noreturn void main(void)
{
	rcc_init();
	gpio_init();
	usart_init();
	spi_init();
	nvic_init();
	tim1_init();
	
	spi_packet_vex m2u, u2m;

	memset(&u2m,0,sizeof(u2m));
	memset(&m2u,0,sizeof(m2u));

	spi_packet_init_u2m(&u2m);
	spi_packet_init_m2u(&m2u);
	
	while(!is_master_ready()) {
		printf("** MASTER WAIT **\n");
	}
	
	printf("[ INIT DONE ]\n");
	uint16_t i = 0;
	for(;;) {
		if (spi_transfer_flag) {
			vex_spi_xfer(&m2u,&u2m);
			printf("i = %d\n",i);
			i++;

			u2m.u2m.motors[1] = m2u.m2u.joysticks[0].b.axis_3;
			
			print_m2u(&m2u);

			spi_transfer_flag = false;
		}
	}
}

#if 0
void rtc_init(void) {
	// Wait for register syncronized flag.
	RTC->CRL &= RTC_CRL_RSF;
	while(!(RTC->CRL & RTC_CRL_RSF));
	
	/*
Configuration procedure:
1. Poll RTOFF, wait until its value goes to ‘1’
2. Set the CNF bit to enter configuration mode
3. Write to one or more RTC registers
4. Clear the CNF bit to exit configuration mode
5. Poll RTOFF, wait until its value goes to ‘1’ 
	to check the end of the write operation.
The write operation only executes when the CNF 
bit is cleared; it takes at least three
RTCCLK cycles to complete.
*/
	
	// Wait for registers to be writeable.
	while(!(RTC->CLR & RTC_CLR_RTOFF));
	
	// enable config mode
	RTC->CLR |= RTC_CLR_CNF;
	
	/** { Config start **/
	
	// disable interrupts.
	RTC->CRH &= ~(RTC_CRH_OWIE
		| RTC_CRH_ALRIE
		| RTC_CRH_SECIE);
	
	// clear flags
	RTC->CRL &= ~(RTC_CRL_OWF
		|RTC_CRL_ALRF
		|RTC_CRL_SECF);
	
	
	
	
	
	/** } config end **/
	
	// commit changes
	RTC->CLR &= RTC_CLR_CNF;
	
	// wait for write to complete
	while(!(RTC->CLR & RTC_CLR_RTOFF));
}
#endif

/* */
__attribute__((interrupt)) void TIM1_CC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_CC1)) {
		spi_transfer_flag = true;
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
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
