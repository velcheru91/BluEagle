/*
 * hal_init.c
 *
 *  Created on: Aug 25, 2016
 *      Author: Venugopal Velcheru
 */
//-----------------------------------------------------------------------------
// Hardware Abstraction Layer Initialization
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
#ifndef PUBLIC_HAL_H_
#include <hal.h>
#endif
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
// Initialization routines which are called only once
// Initialize the system clock
void HAL_Sys_Clk_Init(void)
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN     |
	    		   SYSCTL_RCC_USESYSDIV  | (4 << SYSCTL_RCC_SYSDIV_S) |
				   SYSCTL_RCC_USEPWMDIV  | SYSCTL_RCC_PWMDIV_16;
	// Set GPIO ports to use APB (default)
	SYSCTL_GPIOHBCTL_R = 0;
}

void HAL_Button1_Init(void)
{
	// Enable GPIO peripherals ports
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;
	// Set the direction for Booster pack switches and ADC Input pins
	GPIO_PORTD_DIR_R = HAL_PORT_RESET;
	// unlocking PORTD for enabling functionality of all pins
	GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
	// enabling the functionality of all pins in PORTD
	GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
	// Enable push button pin on PORTD PIN6
	GPIO_PORTD_DEN_R |= (HAL_GPIO_BIT6);
	// enable internal pull-up for push button
	GPIO_PORTD_PUR_R |= (HAL_GPIO_BIT6);
}

void HAL_Button2_Init(void)
{
	// Enable GPIO peripherals ports
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;
	// Set the direction for Booster pack switches and ADC Input pins
	GPIO_PORTD_DIR_R = HAL_PORT_RESET;
	// unlocking PORTD for enabling functionality of all pins
	GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
	// enabling the functionality of all pins in PORTD
	GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
	// Enable push button pin on PORTD PIN6
	GPIO_PORTD_DEN_R |= (HAL_GPIO_BIT7);
	// enable internal pull-up for push button
	GPIO_PORTD_PUR_R |= (HAL_GPIO_BIT7);
}

void HAL_Accelero_Init(void)
{

}

void HAL_RGB_BPACK_Init(void)
{

}

void HAL_Buzzer_Init()
{

}

void HAL_LCD_Init()
{

}

void HAL_Microphone_Init()
{

}

void HAL_Joystick_Init(void)
{

}




void HAL_RGB_LPAD_Init(void)
{
	// With BooserPack attached we can only use RED LED on LaunchPad
	// Enable GPIO peripherals ports
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
	// Configure LED and pushbutton pins
	GPIO_PORTF_DIR_R |= HAL_GPIO_BIT1;
	// enable LED
	GPIO_PORTF_DEN_R |= HAL_GPIO_BIT1;
	// resetting the pin
	GPIO_PORTF_DATA_R &= !HAL_GPIO_BIT1;
}

void HAL_ButtonLPAD_Init(void)
{

}

/*----------------------------------------------------------------*/
// Usage routines which are called as per task rate
bool HAL_Button1_Input(void)
{
	return(((GPIO_PORTD_DATA_R & HAL_GPIO_BIT6)>>6)?false:true);
}

bool HAL_Button2_Input(void)
{
	return(((GPIO_PORTD_DATA_R & HAL_GPIO_BIT7)>>7)?false:true);
}

void HAL_RGB_LPAD_Set(uint8_t red)
{
	(red?(HAL_LPAD_RED_LED_ON):(HAL_LPAD_RED_LED_OFF));
}

void HAL_Buzzer_Set()
{

}
void HAL_Joystick_Input(void)
{

}
void HAL_Application_Start()
{
//	uint16_t raw_input;
//	uint8_t acc_xaxis_output;
//	double scaled_value;
	while(1)
	{
		if(HAL_Button1_Input())
		{
			HAL_RGB_LPAD_Set(1);
		}
		if(HAL_Button2_Input())
		{
			HAL_RGB_LPAD_Set(0);
		}
	}
		//raw_input = hal_ADC0_readSs3();
		//scaled_value = ((((double)raw_input-1000.0)*256.0)/2000.0);
		//acc_xaxis_output = (uint8_t)floor(scaled_value);
		//while (UART0_FR_R & UART_FR_TXFF);
		//UART0_DR_R = (uint8_t)acc_xaxis_output;
		//while (UART1_FR_R & UART_FR_TXFF);
		//UART1_DR_R = (uint8_t)acc_xaxis_output;
		//HAL_LED_SELECT(HAL_LED_PIN_GREEN) = HAL_LED_ON;
		//hal_SW4_press_wait();
		//HAL_LED_SELECT(HAL_LED_PIN_GREEN) = HAL_LED_OFF;
		//HAL_BPAC_BLU_LED_ON;
		//HAL_BPAC_BUZZ_ON;
		//hal_PWM_BUZZ_control(true);

		//hal_SW4_release_wait();
		//UART0_DR_R = 0x33;
		//HAL_BPAC_BLU_LED_OFF;
		//HAL_BPAC_BUZZ_OFF;
		//hal_PWM_BUZZ_control(false);
		//hal_LCD_Senddata(0x46);
//		while(HAL_BPAC_SWJ_STATUS);
//		HAL_LPAD_RED_LED_ON;
//		while(!HAL_BPAC_SWJ_STATUS);
//		HAL_LPAD_RED_LED_OFF;
		//sys_delay_usec(DELAY_1SEC/10);
		//HAL_LED_SELECT(HAL_LED_PIN_GREEN) ^= HAL_LED_TOGGLE;
//	}
}
int16_t hal_ADC0_readSs3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

void HAL_Init(void)
{
	HAL_Sys_Clk_Init();
	HAL_Button1_Init();
	HAL_Button2_Init();
	HAL_RGB_LPAD_Init();

}

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

//// Blocking function that returns only when SW1 is pressed
//void hal_SW1_press_wait(void)
//{
//	while(HAL_LPAD_SW1_STATUS);
//}
//
//// Blocking function that returns only when SW1 is released
//void hal_SW1_release_wait(void)
//{
//	while(!HAL_LPAD_SW1_STATUS);
//}
//
//// Blocking function that returns only when SW2 is pressed
//void hal_SW2_press_wait(void)
//{
//	while(!HAL_LPAD_SW2_STATUS);
//}
//
//// Blocking function that returns only when SW2 is released
//void hal_SW2_release_wait(void)
//{
//	while(HAL_LPAD_SW2_STATUS);
//}
//
//// Blocking function that returns only when SW3 is pressed
//void hal_SW3_press_wait(void)
//{
//	while(HAL_BPAC_SW1_STATUS);
//}
//
//// Blocking function that returns only when SW3 is released
//void hal_SW3_release_wait(void)
//{
//	while(!HAL_BPAC_SW1_STATUS);
//}
//
//// Blocking function that returns only when SW4 is pressed
//void hal_SW4_press_wait(void)
//{
//	while(HAL_BPAC_SW2_STATUS);
//}
//
//// Blocking function that returns only when SW4 is released
//void hal_SW4_release_wait(void)
//{
//	while(!HAL_BPAC_SW2_STATUS);
//}
