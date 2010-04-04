#include "stm32f10x_lib.h"

void main(void)
{
#ifdef DEBUG
	debug();
#endif
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
