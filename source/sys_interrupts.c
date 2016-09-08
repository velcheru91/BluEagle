/*
 * sys_interrupts.c
 *
 *  Created on: Sep 5, 2016
 *      Author: Venugopal Velcheru
 */

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "sys_time.h"
#include "hal_init.h"
#include "sys_interrupts.h"
//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

void _ISR_HAL_UART0_RX_ (void)
{
	uint8_t var_bridge;
	while (UART0_FR_R & UART_FR_RXFE);
	var_bridge = UART0_DR_R;
	while (UART1_FR_R & UART_FR_TXFF);
	UART1_DR_R = var_bridge;
}
void _ISR_HAL_UART1_RX_ (void)
{
	uint8_t var_bridge;
	while (UART1_FR_R & UART_FR_RXFE);
	var_bridge = UART1_DR_R;
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = var_bridge;
}
