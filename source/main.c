/*
 * File name: main.c
 *
 *  Created by					Created on
 *  Venugopal Velcheru			Sep 7, 2016
 */

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Hardware configuration:
// EK-TM4C123GXL Evaluation Board,
// TI Booster Pack,
// CC2650 Bluetooth Module
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#ifndef _STDINT_H_
#include <stdint.h>
#endif
#ifndef __TM4C123GH6PM_H__
#include <tm4c123gh6pm.h>
#endif
#ifndef _STDBOOL
#include <stdbool.h>
#endif
#ifndef PUBLIC_HAL_H_
#include <hal.h>
#endif
//-----------------------------------------------------------------------------
// Declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

int main(void)
{
	HAL_Init();//incipient();
//	hal_Init_Hardware();
	HAL_Application_Start();//reprise();
	return 0;
}
//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
