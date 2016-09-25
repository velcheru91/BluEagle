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
	// Enable GPIO peripherals ports
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB |
	    		     SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD |
				     SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
	SYSCTL_GPIOHBCTL_R = 0;
}

void HAL_Button1_Init(void)
{
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
	// Initializing PB3 for Green LED
	// Enable system clock to PWM modules
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
	// Set the direction for LED control
	GPIO_PORTB_DIR_R |= HAL_GPIO_BIT3;
	// Enable pin 3 on PORT B
	GPIO_PORTB_DEN_R |= HAL_GPIO_BIT3;

	// Initializing PC4 for Blue LED
	// Configure LED pin
	GPIO_PORTC_DIR_R |= HAL_GPIO_BIT4;
	// make bit 1 an outputs
	GPIO_PORTC_DR2R_R |= HAL_GPIO_BIT4;
	// selecting alternate function on PC4d to be PWM
	GPIO_PORTC_AFSEL_R |= HAL_GPIO_BIT4;
	// selecting analog mode on PC4
	GPIO_PORTC_AMSEL_R |= HAL_GPIO_BIT4;
	// configure PC4 to be PWM output pin
	GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6;
	// enable LED
	GPIO_PORTC_DEN_R |= HAL_GPIO_BIT4;
	// resetting the port
	GPIO_PORTC_DATA_R = HAL_PORT_RESET;
	// disabling PWM module before configuration as BLUE LED on BPAC
	PWM0_3_CTL_R &= ~PWM_3_CTL_ENABLE;
	// configuring pulse generator A
	PWM0_3_GENA_R |= PWM_3_GENA_ACTZERO_NONE | PWM_3_GENA_ACTLOAD_ONE |
	                 PWM_3_GENA_ACTCMPAU_NONE | PWM_3_GENA_ACTCMPAD_ZERO |
	                 PWM_3_GENA_ACTCMPBU_NONE | PWM_3_GENA_ACTCMPBD_NONE;
	// writing the load value
	PWM0_3_LOAD_R = HAL_PWM_BUZZ_LOAD;
	// loading the comparator value
	PWM0_3_CMPA_R = HAL_PWM_BUZZ_CMP;
	// enabling the PWM generator
	PWM0_3_CTL_R = PWM_3_CTL_DEBUG;
	// enabling the PWM module
	PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;
}

void HAL_Buzzer_Init(void)
{
	// Enable system clock to PWM modules
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
	// Configure pins for Buzzer
	GPIO_PORTF_DIR_R |= HAL_GPIO_BIT2;
	// selecting alternate function on PF2 to be PWM
	GPIO_PORTF_AFSEL_R |= HAL_GPIO_BIT2;
	// selecting analog mode on PF2
	GPIO_PORTF_AMSEL_R |= HAL_GPIO_BIT2;
	// configure PF2 to be PWM output pin
	GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;
	// enable LED
	GPIO_PORTF_DEN_R |= HAL_GPIO_BIT2;
	// resetting the port
	//GPIO_PORTF_DATA_R = HAL_PORT_RESET;
	// disabling PWM module before configuration as buzzer on BPAC
	PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
	// configuring pulse generator A
	PWM1_3_GENA_R |= PWM_3_GENA_ACTZERO_NONE | PWM_3_GENA_ACTLOAD_ONE |
	                 PWM_3_GENA_ACTCMPAU_NONE | PWM_3_GENA_ACTCMPAD_ZERO |
	                 PWM_3_GENA_ACTCMPBU_NONE | PWM_3_GENA_ACTCMPBD_NONE;
	// writing the load value
	PWM1_3_LOAD_R = HAL_PWM_BUZZ_LOAD;
	// loading the comparator value
	PWM1_3_CMPA_R = HAL_PWM_BUZZ_CMP;
	// enabling the PWM generator
	PWM1_3_CTL_R = PWM_3_CTL_DEBUG;
	// enabling the PWM module
	PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
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

void HAL_Accelero_Input(void)
{

}
void HAL_RGB_BPACK_Set(uint8_t red, uint8_t green, uint8_t blue)
{
	red++; // to avoid compiler warning
	(green?(HAL_BPAC_GRN_LED_ON):(HAL_BPAC_GRN_LED_OFF));
	(blue?(HAL_BPAC_BLU_LED_ON):(HAL_BPAC_BLU_LED_OFF));
}
void HAL_Buzzer_Set(uint8_t buzz)
{
	(buzz?(HAL_BPAC_BUZZ_ON):(HAL_BPAC_BUZZ_OFF));
}

void HAL_Microphone_Input(void)
{

}
void HAL_Joystick_Input(void)
{

}
void HAL_RGB_LPAD_Set(uint8_t red, uint8_t green, uint8_t blue)
{
	(red?(HAL_LPAD_RED_LED_ON):(HAL_LPAD_RED_LED_OFF));
	blue++; // to avoid compiler warning
	green++; // to avoid compiler warning
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
	HAL_RGB_LPAD_Init();HAL_RGB_LPAD_Set(1,0,0);
	HAL_Button1_Init();
	HAL_Button2_Init();
	HAL_RGB_BPACK_Init();
	HAL_Buzzer_Init();
	HAL_Sys_Delay(.5*DELAY_1SEC);HAL_RGB_LPAD_Set(0,0,0);
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
			HAL_RGB_BPACK_Set(0,1,0);
			HAL_Buzzer_Set(1);
		}
		else if(HAL_Button2_Input())
		{
			HAL_RGB_BPACK_Set(0,0,1);
			HAL_Buzzer_Set(1);
		}
		else
		{
			HAL_Buzzer_Set(0);
		}
	}
		//raw_input = hal_ADC0_readSs3();
		//scaled_value = ((((double)raw_input-1000.0)*256.0)/2000.0);
		//acc_xaxis_output = (uint8_t)floor(scaled_value);
		//while (UART0_FR_R & UART_FR_TXFF);
		//UART0_DR_R = (uint8_t)acc_xaxis_output;
		//while (UART1_FR_R & UART_FR_TXFF);
		//UART1_DR_R = (uint8_t)acc_xaxis_output;
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
