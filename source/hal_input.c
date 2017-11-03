/*
 * hal_input.c
 *
 *  Created on: Oct 31, 2017
 *      Author: Venugopal Velcheru
 */

//-----------------------------------------------------------------------------
// Hardware Abstraction Layer Input
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

// Usage routines which are called as per task rate
bool HAL_Button1_Input(void)
{
    return(((GPIO_PORTD_DATA_R & HAL_GPIO_BIT6)>>6)?false:true);
}

bool HAL_Button2_Input(void)
{
    return(((GPIO_PORTD_DATA_R & HAL_GPIO_BIT7)>>7)?false:true);
}

void HAL_Accelero_Input(void)
{

}
void HAL_Microphone_Input(void)
{

}

void HAL_Joystick_Input(uint16_t* x, uint16_t* y, uint32_t* select)
{
    // initiate SS1
    ADC0_PSSI_R |= ADC_PSSI_SS1;
    // wait for conversion done
    while((ADC0_RIS_R & ADC_RIS_INR1)==0){};
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);
    // read first result
    *x = ADC0_SSFIFO1_R>>2;
    // read second result
    *y = ADC0_SSFIFO1_R>>2;
    // return 0(pressed) or 0x10(not pressed)
    //*select = HAL_BPAC_SWJ_STATUS;
    // acknowledge completion
    ADC0_ISC_R = ADC_ISC_IN1;
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
