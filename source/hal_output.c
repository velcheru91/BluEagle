/*
 * hal_output.c
 *
 *  Created on: Oct 31, 2016
 *      Author: Venugopal Velcheru
 */

//-----------------------------------------------------------------------------
// Hardware Abstraction Layer Output
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#ifndef _STDINT_H_
#include <stdint.h>
#endif
#ifndef _STDBOOL
#include <stdbool.h>
#endif
#ifndef _MATH
#include <math.h>
#endif
#ifndef __TM4C123GH6PM_H__
#include <tm4c123gh6pm.h>
#endif
#ifndef BLUEAGLE_HEADER_HAL_H_
#include <hal.h>
#endif
#ifndef PUBLIC_HAL_LCD_H_
#include <hal_lcd.h>
#endif
//-----------------------------------------------------------------------------
// Declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// System reset interrupt routine
void HAL_Reset_ISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void HAL_Sys_Delay(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

/*----------------------------------------------------------------*/

void HAL_RGB_BPACK_Set(uint8_t red, uint8_t green, uint8_t blue)
{
	red++; // to avoid compiler warning
	(green?(HAL_BPAC_GRN_LED_ON):(HAL_BPAC_GRN_LED_OFF));
	(blue?(HAL_BPAC_BLU_LED_ON):(HAL_BPAC_BLU_LED_OFF));
}

void HAL_Buzzer_Set(uint8_t buzz, uint8_t tone)
{
	switch(tone)
	{
	case 0://PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
		   PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.925);//0xEA4;//0xB4AA0;
//		   PWM1_3_CMPA_R = 0xB4AA0;
//		   PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
		break;
	case 1:PWM1_3_CMPA_R = (2500*0.20)-1;
		break;
	case 2:PWM1_3_CMPA_R = (2500*0.30)-1;
		break;
	case 3:PWM1_3_CMPA_R = (2500*0.40)-1;
		break;
	case 4:PWM1_3_CMPA_R = (2500*0.50)-1;
		break;
	case 5:PWM1_3_CMPA_R = (2500*0.60)-1;
		break;
	case 6:PWM1_3_CMPA_R = (2500*0.70)-1;
		break;
	case 7:PWM1_3_CMPA_R = (2500*0.80)-1;
		break;
	case 90://PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
		     PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);//0x1387;
//			 PWM1_3_CMPA_R = 0xAFCE4;
//		     PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
		break;
	case 180://PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
		      PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.95);//0x9C3;
//			  PWM1_3_CMPA_R = 0xB985C;
//		      PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
		break;
	default:PWM1_3_CMPA_R = HAL_PWM_BUZZ_CMP;
		break;
	}
	(buzz?(HAL_BPAC_BUZZ_ON):(HAL_BPAC_BUZZ_OFF));
}

void HAL_RGB_LPAD_Set(uint8_t red, uint8_t green, uint8_t blue)
{
	(red?(HAL_LPAD_RED_LED_ON):(HAL_LPAD_RED_LED_OFF));
	blue++; // to avoid compiler warning
	green++; // to avoid compiler warning
}

void HAL_LPAD_UART_Write(uint8_t data)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = data;

}

void HAL_LPAD_UART_Read(uint8_t* data)
{
	while (UART0_FR_R & UART_FR_RXFE);
	*data = UART0_DR_R;
}

//int16_t hal_ADC0_readSs3()
//{
//    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
//               // wait until SS3 is not busy
//    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
//}

//void HAL_Application_Start()
//{

//}
//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

void _ISR_HAL_UART0_RX_ (void)
{
/*	uint8_t var_bridge;
	while (UART0_FR_R & UART_FR_RXFE);
	var_bridge = UART0_DR_R;
	while (UART1_FR_R & UART_FR_TXFF);
	UART1_DR_R = var_bridge;*/
}
void _ISR_HAL_UART1_RX_ (void)
{
/*	uint8_t var_bridge;
	while (UART1_FR_R & UART_FR_RXFE);
	var_bridge = UART1_DR_R;
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = var_bridge;*/
}

void SysTick_interrupt (void)
{
    if (TM1.active)
        TM1.count++;
    if (TM2.active)
        TM2.count++;
    if (TM3.active)
        TM3.count++;
    if (TM4.active)
        TM4.count++;

}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
