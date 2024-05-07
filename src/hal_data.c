#include "hal_data.h"


void __BKPT(int number)
{
	FSP_PARAMETER_NOT_USED(number);
	__asm volatile ( "BRK" );
}

void __NOP(void)
{
	__asm volatile ( "NOP" );
}
