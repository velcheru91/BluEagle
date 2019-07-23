/*
 * hal_init.c
 *
 *  Created on: Oct 31, 2017
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
#ifndef BLUEAGLE_HEADER_HAL_H_
#include <hal.h>
#endif
#ifndef PUBLIC_HAL_LCD_H_
#include <hal_lcd.h>
#endif
//-----------------------------------------------------------------------------
// Declarations
//-----------------------------------------------------------------------------

TIMER TM1;
TIMER TM2;
TIMER TM3;
TIMER TM4;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

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
    PWM1_3_LOAD_R = HAL_PWM_BUZZ_LOAD;//0xC34F;//0xC3500;//
//  PWM1_3_LOAD_R = 0xC3500;//HAL_PWM_BUZZ_LOAD;
    // loading the comparator value
    PWM1_3_CMPA_R = HAL_PWM_BUZZ_CMP;
    // enabling the PWM generator
    PWM1_3_CTL_R = PWM_3_CTL_DEBUG;
    // enabling the PWM module
    PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
}

void HAL_Servo1_Init(void)
{
    // Enable system clock to PWM modules
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    // Configure pins for Buzzer
    GPIO_PORTE_DIR_R |= HAL_GPIO_BIT5;
    // make bit 1 an outputs
    GPIO_PORTE_DR2R_R |= HAL_GPIO_BIT5;
    // selecting alternate function on PF2 to be PWM
    GPIO_PORTE_AFSEL_R |= HAL_GPIO_BIT5;
    // selecting analog mode on PF2
    GPIO_PORTE_AMSEL_R |= HAL_GPIO_BIT5;
    // configure PF2 to be PWM output pin
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE5_M1PWM3;
    // enable LED
    GPIO_PORTE_DEN_R |= HAL_GPIO_BIT5;
    // resetting the port
    GPIO_PORTE_DATA_R = HAL_PORT_RESET;
    // disabling PWM module before configuration as buzzer on BPAC
    PWM1_1_CTL_R &= ~PWM_2_CTL_ENABLE;
    // configuring pulse generator A
    PWM1_1_GENA_R |= PWM_2_GENA_ACTZERO_NONE | PWM_2_GENA_ACTLOAD_ONE |
                     PWM_2_GENA_ACTCMPAU_NONE | PWM_2_GENA_ACTCMPAD_ZERO |
                     PWM_2_GENA_ACTCMPBU_NONE | PWM_2_GENA_ACTCMPBD_NONE;
    // writing the load value
    PWM1_1_LOAD_R = HAL_PWM_BUZZ_LOAD;//0xC34F;//0xC3500;//
    // loading the comparator value
    PWM1_1_CMPA_R = HAL_PWM_BUZZ_CMP;
    // enabling the PWM generator
    PWM1_1_CTL_R = PWM_1_CTL_DEBUG;
    // enabling the PWM module
    PWM1_1_CTL_R |= PWM_1_CTL_ENABLE;
}

void HAL_Servo2_Init(void)
{
    // Enable system clock to PWM modules
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    // Configure pins for Buzzer
    GPIO_PORTC_DIR_R |= HAL_GPIO_BIT5;
    // make bit 1 an outputs
    GPIO_PORTC_DR2R_R |= HAL_GPIO_BIT5;
    // selecting alternate function on PF2 to be PWM
    GPIO_PORTC_AFSEL_R |= HAL_GPIO_BIT5;
    // selecting analog mode on PF2
    GPIO_PORTC_AMSEL_R |= HAL_GPIO_BIT5;
    // configure PF2 to be PWM output pin
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_M0PWM7;
    // enable LED
    GPIO_PORTC_DEN_R |= HAL_GPIO_BIT5;
    // resetting the port
    GPIO_PORTC_DATA_R = HAL_PORT_RESET;
    // disabling PWM module before configuration as buzzer on BPAC
    PWM0_3_CTL_R &= ~PWM_3_CTL_ENABLE;
    // configuring pulse generator A
    PWM0_3_GENA_R |= PWM_2_GENA_ACTZERO_NONE | PWM_2_GENA_ACTLOAD_ONE |
                     PWM_2_GENA_ACTCMPAU_NONE | PWM_2_GENA_ACTCMPAD_ZERO |
                     PWM_2_GENA_ACTCMPBU_NONE | PWM_2_GENA_ACTCMPBD_NONE;
    // writing the load value
    PWM0_3_LOAD_R = HAL_PWM_BUZZ_LOAD;//0xC34F;//0xC3500;//
    // loading the comparator value
    PWM0_3_CMPA_R = HAL_PWM_BUZZ_CMP;
    // enabling the PWM generator
    PWM0_3_CTL_R = PWM_0_CTL_DEBUG;
    // enabling the PWM module
    PWM0_3_CTL_R |= PWM_0_CTL_ENABLE;
}

//void static adcinit(void)
//{
//}

void HAL_Microphone_Init()
{

}

void HAL_Joystick_Init(void)
{
    // Set the direction for Joystick X axis PB5
    GPIO_PORTB_DIR_R &= ~(HAL_GPIO_BIT5);
    // turn off digital operation on pins PB5
    GPIO_PORTB_DEN_R &= ~HAL_GPIO_BIT5;
    // Alternate function selection for Joystick X axis
    GPIO_PORTB_AFSEL_R |= HAL_GPIO_BIT5;
    // turn on analog operation on pins PB5 for AIN11
    GPIO_PORTB_AMSEL_R |= HAL_GPIO_BIT5;

    // Set the direction for Booster pack switches and ADC Input pins
    GPIO_PORTD_DIR_R = HAL_PORT_RESET;
    // unlocking PORTD for enabling functionality of all pins
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
    // enabling the functionality of all pins in PORTD
    GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
    // selecting alternate function on PortD pins to be ADC Input
    GPIO_PORTD_AFSEL_R |= HAL_GPIO_BIT3;
    // turn off digital operation on pins PD3
    GPIO_PORTD_DEN_R &= ~HAL_GPIO_BIT3;
    // turn on analog operation on pins PD3
    GPIO_PORTD_AMSEL_R |= HAL_GPIO_BIT3;

    // unlocking PORTE for enabling functionality of all pins
    GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY;
    // enabling the functionality of all pins in PORTD
    GPIO_PORTE_CR_R = HAL_PORT_ENABLE;
    // Configure Joystick select pin PE4
    GPIO_PORTE_DIR_R &= ~HAL_GPIO_BIT4;
    // make bit 1 an outputs
    GPIO_PORTE_DR2R_R |= HAL_GPIO_BIT4;
    // enable internal pull-up for push button
    GPIO_PORTE_PUR_R |= HAL_GPIO_BIT4;
    // enable PortE
    GPIO_PORTE_DEN_R |= HAL_GPIO_BIT4;

    // Enable system clock to ADC modules
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    // Allow time for clock to stabilize
    while((SYSCTL_PRADC_R&0x01) == 0){};
    // GPIO initialization in more specific functions
    // select PLL as the time base (not needed, since default value)
    ADC0_CC_R = ADC_CC_CS_SYSPLL;
    //clear max sample rate field
    ADC0_PC_R &= ~ADC_SPC_PHASE_M;
    //configure for 125K samples/sec
    ADC0_PC_R |= ADC_SPC_PHASE_22_5;
    // Sequencer 3 is lowest priority
    ADC0_SSPRI_R = 0x3210;

    // disable sample sequencer 1 (SS1) for programming
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;
    // select SS1 bit in ADCPSSI as trigger
    ADC0_EMUX_R &= ~ADC_EMUX_EM1_M;
    // set first and second samples to AIN4 and AIN11
    ADC0_SSMUX1_R = 0x004B;
    // no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
    ADC0_SSCTL1_R = ADC_SSCTL1_IE1 | ADC_SSCTL1_END1;
    // disable SS1 interrupts
    ADC0_IM_R &= ~ADC_IM_MASK1;
    // enable sample sequencer 1
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;
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

void HAL_BT_UART_Init(void)
{
    //Enable system clock to UART1
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    // Set the direction for UART1 RX and Tx
    GPIO_PORTB_DIR_R |= (HAL_GPIO_BIT1);
    GPIO_PORTB_DIR_R &= ~(HAL_GPIO_BIT0);
    // Enable UART1 and SPI2 pins on PORT B
    GPIO_PORTB_DEN_R |= (HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
    // Alternate function selection for UART 1
    GPIO_PORTB_AFSEL_R |= (HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
    // Port Control selection for UART and SPI pins
    GPIO_PORTB_PCTL_R = (GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX);

    // turn-off UART1 to allow safe programming
    UART1_CTL_R = 0;
    // use system clock (40 MHz)
    UART1_CC_R = UART_CC_CS_SYSCLK;
    // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_IBRD_R = 260;//21;
    // round(fract(r)*64)=45
    UART1_FBRD_R = 27;//45;
    // configure for 8N1 w/ 16-level FIFO disabled
    UART1_LCRH_R = UART_LCRH_WLEN_8;
    // enable TX, RX, and module
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enabling interrupt on UART1 RX
    UART1_IM_R = UART_IM_RXIM;
    // setting UART1 bit in vector table
    NVIC_EN0_R = 1<<6;
}

void HAL_LPAD_UART_Init(void)
{
    //Enable system clock to UART0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    // Set the direction for UART0 RX and Tx
    GPIO_PORTA_DIR_R |= HAL_GPIO_BIT1;
    GPIO_PORTA_DIR_R &= ~HAL_GPIO_BIT0;
    // Enable UART0 pins on PORT A
    GPIO_PORTA_DEN_R |= (HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
    // Alternate function selection for UART 0
    GPIO_PORTA_AFSEL_R |= HAL_GPIO_BIT1 | HAL_GPIO_BIT0;
    // Port Control selection for UART pins
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // turn-off UART0 to allow safe programming
    UART0_CTL_R = 0;
    // use system clock (40 MHz)
    UART0_CC_R = UART_CC_CS_SYSCLK;
    // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_IBRD_R = 260;//21;
    // round(fract(r)*64)=45
    UART0_FBRD_R = 27;//45;
    // configure for 8N1 w/ 16-level FIFO disabled
    UART0_LCRH_R = UART_LCRH_WLEN_8;
    // enable TX, RX, and module
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enabling interrupt on UART0 RX
    UART0_IM_R = UART_IM_RXIM;
    // setting UART1 bit in vector table
    NVIC_EN0_R = 1<<5;
}


void HAL_Switch_Init(void)
{
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    // enabling the functionality of all pins in PORTD
    GPIO_PORTF_CR_R = HAL_PORT_ENABLE;
    // enable LED
    GPIO_PORTF_DIR_R &= ~(HAL_GPIO_BIT0 | HAL_GPIO_BIT4);
    // selecting alternate function on PF2 to be PWM
    GPIO_PORTF_PUR_R |= (HAL_GPIO_BIT0 | HAL_GPIO_BIT4);
//    GPIO_PORTF_AFSEL_R |= HAL_GPIO_BIT2;
    GPIO_PORTF_DEN_R |= (HAL_GPIO_BIT0 | HAL_GPIO_BIT4);
    // selecting analog mode on PF2
//    GPIO_PORTF_AMSEL_R |= HAL_GPIO_BIT2;
    // configure PF2 to be PWM output pin
//    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;

}

void HAL_Init(void)
{
    HAL_Sys_Clk_Init();
    SysTick_Init();
//  HAL_RGB_LPAD_Init();
//  HAL_RGB_LPAD_Set(1,0,0);
//  HAL_Button1_Init();
//  HAL_Button2_Init();
    HAL_Switch_Init();
    HAL_RGB_BPACK_Init();
//  HAL_Sys_Delay(DELAY_1SEC);
//    HAL_Buzzer_Init();
//  HAL_Servo1_Init();
//  HAL_Servo2_Init();
    HAL_Joystick_Init();
//  HAL_LPAD_UART_Init();
    //HAL_LCD_Init();
    //HAL_Sys_Delay(.5*DELAY_1SEC);
//  HAL_RGB_LPAD_Set(0,0,0);
}

// Initialize Systick timer
void SysTick_Init(void)
{
    NVIC_ST_CTRL_R =0;
    NVIC_ST_RELOAD_R = 0x9C3F; //Frequency = Clock frequency/[n(Value to be loaded into RELOAD register)+1]
    NVIC_ST_CURRENT_R = 0;       // For systick to fire interrupt at 1 ms rate, n = 0x9C3F
    NVIC_ST_CTRL_R = 0x07;

    TM1.load = 3;
    TM1.active = true;
    TM2.load = 3;
    TM2.active = true;
    TM3.load = 3;
    TM3.active = true;
    TM4.load = 3;
    TM4.active = true;
}

//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
