
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

static void spi_init(void)
{
	/* Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* GPIO */
	/* 
	Alternate function SPI1_REMAP = 0 SPI1_REMAP = 1
	SPI1_NSS             PA4           PA15
	SPI1_SCK             PA5           PB3
	SPI1_MISO            PA6           PB4
	SPI1_MOSI            PA7           PB5 
	*/

	GPIO_InitTypeDef GPIO_param;

	GPIO_param.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_param.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_param.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_param);	

	GPIO_param.GPIO_Pin = GPIO_Pin_6;
	GPIO_param.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_param);

	/* SPI */
	SPI_InitTypeDef SPI_param;

	SPI_param.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_param.SPI_Mode = SPI_Mode_Master;
	SPI_param.SPI_DataSize = SPI_DataSize_16b;
	SPI_param.SPI_CPOL = SPI_CPOL_Low;
	SPI_param.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_param.SPI_NSS = SPI_NSS_Soft;
	SPI_param.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_param.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_param.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_param);

	SPI_Cmd(SPI1, ENABLE);

	/* Master Detect Lines: PE{3,4} */
	GPIO_param.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_param.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_param);

	/* Slave select : PE0 */
	GPIO_param.GPIO_Pin = GPIO_Pin_0;
	GPIO_param.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_param.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_param);

	/* Famed "RTS" Pin: PA11 */
	GPIO_param.GPIO_Pin = GPIO_Pin_11;
	GPIO_param.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_param.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_param);	

	/* Master Connect Routines */
	uint8_t i;
	uint16_t init_data[8];

	// Slave Select?
	GPIO_SetBits(GPIOE, GPIO_Pin_0);

	// Interesting...
	for(i = 0; i < 8; i++)
		init_data[i] = SPI_I2S_ReceiveData(SPI1);
}

void spi_vex_xfer(spi_packet_vex *m2u, spi_packet_vex *u2m)
{
	static uint8_t packet_num = 0;
	uint8_t gap = 0;
	volatile uint16_t d = 0;
	uint8_t i = 0;
	
	u2m->u2m.packet_num = packet_num;

	GPIO_SetBits(GPIOA, GPIO_Pin_11); // "RTS" high

	for (i = 0; i < SPI_PACKET_LEN; i++) {                  
		GPIO_ResetBits(GPIOE, GPIO_Pin_0); // Slave Select
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, u2m->w[i]); 

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		m2u->w[i] = SPI_I2S_ReceiveData(SPI1);

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_SetBits(GPIOE, GPIO_Pin_0); // Slave Select
		for (d = 0; d < 150; d++); //15us
		gap++;
		if (gap == 4) { //put a gap after 4 bytes xfered
			for (d = 0; d < 1000; d++); //210us
			GPIO_ResetBits(GPIOA, GPIO_Pin_11); //RTS low
			gap = 0;
		}
	}

	packet_num++;   
}

bool is_master_ready(void)
{
	// master is ready when both input lines are low.
	return !GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) &&
		!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);
}

void vex_spi_packet_init_m2u(spi_packet_vex *m2u)
{

}

void vex_spi_packet_init_u2m(spi_packet_vex *u2m)
{
	u2m->u2m.sync = SYNC_MAGIC;
	u2m->u2m.version = 1;

	uint8_t i;
	for(i = 0; i < MOTOR_CT; i++) {
		u2m->u2m.motors[i] = 127;
	}
}

__noreturn void main(void)
{
	rcc_init();
	gpio_init();
	usart_init();
	spi_init();

	spi_packet_vex m2u, u2m;

	memset(m2u,0,sizeof(m2u));
	memset(u2m,0,sizeof(u2m));

	vex_spi_packet_init_u2m(&u2m);
	vex_spi_packet_init_m2u(&m2u);

	while(!is_master_ready()) {
		usart1_puts("Waiting for master\n");
	}

	for(;;) {
		usart1_puts("HELLOOOOO\n");
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
