/*
 * sys_interrupts.h
 *
 *  Created on: Sep 5, 2016
 *      Author: code_it
 */

#ifndef PUBLIC_SYS_INTERRUPTS_H_
#define PUBLIC_SYS_INTERRUPTS_H_


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Device prototype, function and method decleration
//-----------------------------------------------------------------------------
extern void _ISR_HAL_UART0_RX_ (void);
extern void _ISR_HAL_UART1_RX_ (void);

#endif /* PUBLIC_SYS_INTERRUPTS_H_ */
