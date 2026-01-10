// Runs on either TM4C123
// Start project to Lab 1.  Take sensor readings, process the data,
// and output the results.  Specifically, this program will
// measure steps using the accelerometer, audio noise using
// microphone, and light intensity using the light sensor.
// Daniel and Jonathan Valvano
// February 3, 2016

/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2016

 Copyright 2016 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

//  J1   J3               J4   J2
// [ 1] [21]             [40] [20]
// [ 2] [22]             [39] [19]
// [ 3] [23]             [38] [18]
// [ 4] [24]             [37] [17]
// [ 5] [25]             [36] [16]
// [ 6] [26]             [35] [15]
// [ 7] [27]             [34] [14]
// [ 8] [28]             [33] [13]
// [ 9] [29]             [32] [12]
// [10] [30]             [31] [11]

// +3.3V connected to J1.1 (power)
// joystick horizontal (X) connected to J1.2 (analog)
// UART from BoosterPack to LaunchPad connected to J1.3 (UART)
// UART from LaunchPad to BoosterPack connected to J1.4 (UART)
// joystick Select button connected to J1.5 (digital)
// microphone connected to J1.6 (analog)
// LCD SPI clock connected to J1.7 (SPI)
// ambient light (OPT3001) interrupt connected to J1.8 (digital)
// ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL connected to J1.9 (I2C)
// ambient light (OPT3001) and temperature sensor (TMP006) I2C SDA connected to J1.10 (I2C)

// temperature sensor (TMP006) interrupt connected to J2.11 (digital)
// nothing connected to J2.12 (SPI CS_Other)
// LCD SPI CS connected to J2.13 (SPI)
// nothing connected to J2.14 (SPI MISO)
// LCD SPI data connected to J2.15 (SPI)
// nothing connected to J2.16 (reset)
// LCD !RST connected to J2.17 (digital)
// nothing connected to J2.18 (SPI CS_Wireless)
// servo PWM connected to J2.19 (PWM)
// GND connected to J2.20 (ground)

// +5V connected to J3.21 (power)
// GND connected to J3.22 (ground)
// accelerometer X connected to J3.23 (analog)
// accelerometer Y connected to J3.24 (analog)
// accelerometer Z connected to J3.25 (analog)
// joystick vertical (Y) connected to J3.26 (analog)
// nothing connected to J3.27 (I2S WS)
// nothing connected to J3.28 (I2S SCLK)
// nothing connected to J3.29 (I2S SDout)
// nothing connected to J3.30 (I2S SDin)

// LCD RS connected to J4.31 (digital)
// user Button2 (bottom) connected to J4.32 (digital)
// user Button1 (top) connected to J4.33 (digital)
// gator hole switch connected to J4.34 (digital)
// nothing connected to J4.35
// nothing connected to J4.36
// RGB LED blue connected to J4.37 (PWM)
// RGB LED green connected to J4.38 (PWM)
// RGB LED red (jumper up) or LCD backlight (jumper down) connected to J4.39 (PWM)
// buzzer connected to J4.40 (PWM)
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include "BSP.h"
#include "tm4c123gh6pm.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <../driverlib/sysctl.h>
#include <../driverlib/gpio.h>
#include <../driverlib/interrupt.h>
#include <inc/hw_memmap.h>

//#include "wdtask.h"
#define mainTASK_A_PRIORITY                 (tskIDLE_PRIORITY + 1)
#define mainTASK_B_PRIORITY                 (tskIDLE_PRIORITY + 4)
#define MAXRETRIES              5  // number of receive attempts before giving up

QueueHandle_t xQueue = NULL;
QueueHandle_t yQueue = NULL;
//void DisableInterrupts(void); // Disable interrupts
//void EnableInterrupts(void);  // Enable interrupts
//long StartCritical (void);    // previous I bit, disable interrupts
//void EndCritical(long sr);    // restore I bit to previous value
//void WaitForInterrupt(void);  // low power mode

static uint32_t ClockFrequency = 16000000; // cycles/second
void DisableInterrupts(void)
{
 __asm  ("    CPSID  I\n"
    "    BX     LR\n");
}
void EnableInterrupts(void)
{
 __asm  ("    CPSIE  I\n"
    "    BX     LR\n");
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
/*void waitMicrosecond(uint32_t us)
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
}*/
/**
 *  Microcontroller's part revision number.
 *  This variable is automatically initialized
 *  during startup and may be declared as an external
 *  variable where needed.
 */
int8_t sysctl_mcu_revision = MCU_REV_NOT_KNOWN_YET;
/* Declared in sysctl.c */
//extern int8_t sysctl_mcu_revision;
/* Microcontroller's part revision number is an important
 * piece of information, essential for implementation of
 * workarounds of known hardware bugs, described in the
 * Tiva(TM) C Series TM4C123x Microcontrollers Silicon Revisions 6 and 7,
 *     Silicon Errata
 * (http://www.ti.com/lit/er/spmz849c/spmz849c.pdf).
 *
 * @return microcontroller's part revision number (between 1 and 7 or a negative value if unknown)
 */
int8_t BSP_SysCtl_mcuRev(void)
{
     /* The revision number can be determined from the DID0
     * register. See page 238 of the Data Sheet
     * for more details.*/
    uint8_t minor;
    uint8_t major;
    if ( MCU_REV_NOT_KNOWN_YET == sysctl_mcu_revision )
    {
        minor = (uint8_t) (SYSCTL_DID0_R & 0x000000FF);
        major = (uint8_t) ((SYSCTL_DID1_R & 0x0000FF00) >> 8);
        sysctl_mcu_revision = MCU_REV_UNKNOWN;
        if ( 0==major && minor<=3 )
        {
          sysctl_mcu_revision = minor +1;
        }
        if ( 1==major && minor<=2 )
        {
          sysctl_mcu_revision = minor + 5;
        }
    }
    return sysctl_mcu_revision;
}
// ------------BSP_Clock_InitFastest------------
// Configure the system clock to run at the fastest
// and most accurate settings.  For example, if the
// LaunchPad has a crystal, it should be used here.
// Call BSP_Clock_GetFreq() to get the current system
// clock frequency for the LaunchPad.
// Input: none
// Output: none
void BSP_Clock_InitFastest(void){
  // 0) configure the system to use RCC2 for advanced features
  //    such as 400 MHz PLL and non-integer System Clock Divisor
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;// configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;// clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;// configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  // 4) set the desired system divider and the system divider least significant bit
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) // clear system clock divider field
                  + (4<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
  ClockFrequency = 80000000;
  // Enable GPIO ports A,B,E,F peripherals
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
}
// ------------BSP_Clock_InitFastest------------
// Configure the system clock to run at the fastest
// and most accurate settings.  For example, if the
// LaunchPad has a crystal, it should be used here.
// Call BSP_Clock_GetFreq() to get the current system
// clock frequency for the LaunchPad.
// Input: none
// Output: none
void BSP_Clock_Init_50Mz(void){
  // 0) configure the system to use RCC2 for advanced features
  //    such as 400 MHz PLL and non-integer System Clock Divisor
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;// configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;// clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;// configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  // 4) set the desired system divider and the system divider least significant bit
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) // clear system clock divider field
                  + (7<<22);      // configure for 50 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
  ClockFrequency = 50000000;
  // Enable GPIO ports A,B,E,F peripherals
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
}
// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 20 nsec for 50 MHz clock)
//void SysTick_Wait(uint32_t delay){
//  volatile uint32_t elapsedTime;
//  uint32_t startTime = NVIC_ST_CURRENT_R;
//  do{
//    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
//  }
//  while(elapsedTime <= delay);
//}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(uint32_t delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// Time delay using busy wait.
// This assumes 80 MHz system clock.
void SysTick80_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms (assumes 80MHz clock)
  }
}

uint32_t Time;
// start the stopwatch
void SysTick_Start(void){
  Time = NVIC_ST_CURRENT_R;
}
// end the stopwatch, and return elased time in bus cycles
uint32_t SysTick_Stop(void){
  return (Time-NVIC_ST_CURRENT_R)&0x00FFFFFF;
}
// ------------BSP_Button1_Init------------
// Initialize a GPIO pin for input, which corresponds
// with BoosterPack pin J4.33.
// Input: none
// Output: none
void BSP_Button1_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x00000008; // 1) activate clock for Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};// allow time for clock to stabilize
                                   // 2) no need to unlock PD6
  GPIO_PORTD_AMSEL_R &= ~0x40;     // 3) disable analog on PD6
                                   // 4) configure PD6 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xF0FFFFFF)+0x00000000;
  GPIO_PORTD_DIR_R &= ~0x40;       // 5) make PD6 input
  GPIO_PORTD_AFSEL_R &= ~0x40;     // 6) disable alt funct on PD6
  GPIO_PORTD_PUR_R &= ~0x40;       // disable pull-up on PD6
  GPIO_PORTD_DEN_R |= 0x40;        // 7) enable digital I/O on PD6
}
// ------------BSP_Button1_Input------------
// Read and return the immediate status of Button1.
// Button de-bouncing is not considered.
// Input: none
// Output: non-zero if Button1 unpressed
//         zero if Button1 pressed
// Assumes: BSP_Button1_Init() has been called
#define BUTTON1   (*((volatile uint32_t *)0x40007100))  /* PD6 */
uint8_t BSP_Button1_Input(void){
  return BUTTON1;                  // return 0(pressed) or 0x40(not pressed)
}

// ------------BSP_Button2_Init------------
// Initialize a GPIO pin for input, which corresponds
// with BoosterPack pin J4.32.
// Input: none
// Output: none
void BSP_Button2_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x00000008; // 1) activate clock for Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};// allow time for clock to stabilize
  GPIO_PORTD_LOCK_R = 0x4C4F434B;  // 2) unlock GPIO Port D
  GPIO_PORTD_CR_R = 0xFF;          // allow changes to PD7-0
  GPIO_PORTD_AMSEL_R &= ~0x80;     // 3) disable analog on PD7
                                   // 4) configure PD7 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0x0FFFFFFF)+0x00000000;
  GPIO_PORTD_DIR_R &= ~0x80;       // 5) make PD7 input
  GPIO_PORTD_AFSEL_R &= ~0x80;     // 6) disable alt funct on PD7
  GPIO_PORTD_PUR_R &= ~0x80;       // disable pull-up on PD7
  GPIO_PORTD_DEN_R |= 0x80;        // 7) enable digital I/O on PD7
}

// ------------BSP_Button2_Input------------
// Read and return the immediate status of Button2.
// Button de-bouncing is not considered.
// Input: none
// Output: non-zero if Button2 unpressed
//         zero if Button2 pressed
// Assumes: BSP_Button2_Init() has been called
#define BUTTON2   (*((volatile uint32_t *)0x40007200))  /* PD7 */
uint8_t BSP_Button2_Input(void){
  return BUTTON2;                  // return 0(pressed) or 0x80(not pressed)
}
// There are six analog inputs on the Educational BoosterPack MKII:
// microphone (J1.6/PE5/AIN8)
// joystick X (J1.2/PB5/AIN11) and Y (J3.26/PD3/AIN4)
// accelerometer X (J3.23/PD0/AIN7), Y (J3.24/PD1/AIN6), and Z (J3.25/PD2/AIN5)
// All six initialization functions can use this general ADC
// initialization.  The joystick uses sample sequencer 1,
// the accelerometer sample sequencer 2, and the microphone
// uses sample sequencer 3.
static void adcinit(void){
  SYSCTL_RCGCADC_R |= 0x00000001;  // 1) activate ADC0
  while((SYSCTL_PRADC_R&0x01) == 0){};// 2) allow time for clock to stabilize
                                   // 3-7) GPIO initialization in more specific functions
  ADC0_PC_R &= ~0xF;               // 8) clear max sample rate field
  ADC0_PC_R |= 0x1;                //    configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;           // 9) Sequencer 3 is lowest priority
                                   // 10-15) sample sequencer initialization in more specific functions
}
// ------------BSP_Joystick_Init------------
// Initialize a GPIO pin for input, which corresponds
// with BoosterPack pin J1.5 (Select button).
// Initialize two ADC pins, which correspond with
// BoosterPack pins J1.2 (X) and J3.26 (Y).
// Input: none
// Output: none
void BSP_Joystick_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x0000001A; // 1) activate clock for Ports E, D, and B
  while((SYSCTL_PRGPIO_R&0x1A) != 0x1A){};// allow time for clocks to stabilize
                                   // 2) no need to unlock PE4, PD3, or PB5
  GPIO_PORTE_AMSEL_R &= ~0x10;     // 3a) disable analog on PE4
  GPIO_PORTD_AMSEL_R |= 0x08;      // 3b) enable analog on PD3
  GPIO_PORTB_AMSEL_R |= 0x20;      // 3c) enable analog on PB5
                                   // 4) configure PE4 as GPIO
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFF0FFFF)+0x00000000;
  GPIO_PORTE_DIR_R &= ~0x10;       // 5a) make PE4 input
  GPIO_PORTD_DIR_R &= ~0x08;       // 5b) make PD3 input
  GPIO_PORTB_DIR_R &= ~0x20;       // 5c) make PB5 input
  GPIO_PORTE_AFSEL_R &= ~0x10;     // 6a) disable alt funct on PE4
  GPIO_PORTD_AFSEL_R |= 0x08;      // 6b) enable alt funct on PD3
  GPIO_PORTB_AFSEL_R |= 0x20;      // 6c) enable alt funct on PB5
  GPIO_PORTE_DEN_R |= 0x10;        // 7a) enable digital I/O on PE4
  GPIO_PORTD_DEN_R &= ~0x08;       // 7b) enable analog functionality on PD3
  GPIO_PORTB_DEN_R &= ~0x20;       // 7c) enable analog functionality on PB5
  adcinit();                       // 8-9) general ADC initialization
  ADC0_ACTSS_R &= ~0x0002;         // 10) disable sample sequencer 1
  ADC0_EMUX_R &= ~0x00F0;          // 11) seq1 is software trigger
  ADC0_SSMUX1_R = 0x004B;          // 12) set channels for SS1
  ADC0_SSCTL1_R = 0x0060;          // 13) no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
  ADC0_IM_R &= ~0x0002;            // 14) disable SS1 interrupts
  ADC0_ACTSS_R |= 0x0002;          // 15) enable sample sequencer 1
}

// ------------BSP_Joystick_Input------------
// Read and return the immediate status of the
// joystick.  Button de-bouncing for the Select
// button is not considered.  The joystick X- and
// Y-positions are returned as 10-bit numbers,
// even if the ADC on the LaunchPad is more precise.
// Input: x is pointer to store X-position (0 to 1023)
//        y is pointer to store Y-position (0 to 1023)
//        select is pointer to store Select status (0 if pressed)
// Output: none
// Assumes: BSP_Joystick_Init() has been called
#define SELECT    (*((volatile uint32_t *)0x40024040))  /* PE4 */
void BSP_Joystick_Input(uint16_t *x, uint16_t *y, uint8_t *select){
  ADC0_PSSI_R = 0x0002;            // 1) initiate SS1
  while((ADC0_RIS_R&0x02)==0){};   // 2) wait for conversion done
  *x = ADC0_SSFIFO1_R>>2;          // 3a) read first result
  *y = ADC0_SSFIFO1_R>>2;          // 3b) read second result
  *select = SELECT;                // return 0(pressed) or 0x10(not pressed)
  ADC0_ISC_R = 0x0002;             // 4) acknowledge completion
}

// ------------BSP_RGB_Init------------
// Initialize the GPIO pins for output which
// correspond to pins PF_1 (red),
// PF_3 (green), and PF_2 (blue).  Instead of using
// PWM or timer modules, this configuration uses just
// digital fully on or fully off.
// non-zero is fully on.
// 0 is fully off.
// Input: red is initial status for red
//        green is initial status for green
//        blue is initial status for blue
// Output: none
void BSP_RGB_onboard_Init(int red, int green, int blue){
  // Enable the GPIO port that is used for the on-board LED.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  // Check if the peripheral access is enabled.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
  // Enable the GPIO pin for the LED (PortF pins).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (red == 1?GPIO_PIN_1:0));
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (blue == 1?GPIO_PIN_2:0));
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (green == 1?GPIO_PIN_3:0));
}

void LED_all_on_off(int red, int green, int blue)
{
  // Turn on the LED.
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, (red == 1?GPIO_PIN_1:0));
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, (blue == 1?GPIO_PIN_2:0));
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, (green == 1?GPIO_PIN_3:0));
}
void LED_red_on_off(int val)
{
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, (val == 1?GPIO_PIN_1:0));
}
void LED_green_on_off(int val)
{
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, (val == 1?GPIO_PIN_3:0));
}
void LED_blue_on_off(int val)
{
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, (val == 1?GPIO_PIN_2:0));
}
// ------------BSP_RGB_D_Init------------
// Initialize the GPIO pins for output which
// correspond with BoosterPack pins J4.39 (red),
// J4.38 (green), and J4.37 (blue).  Instead of using
// PWM or timer modules, this configuration uses just
// digital fully on or fully off.
// non-zero is fully on.
// 0 is fully off.
// Input: red is initial status for red
//        green is initial status for green
//        blue is initial status for blue
// Output: none
#define RED       (*((volatile uint32_t *)0x40025020))  /* PF3 */
#define GREEN     (*((volatile uint32_t *)0x40005020))  /* PB3 */
#define BLUE      (*((volatile uint32_t *)0x40006040))  /* PC4 */
// ------------BSP_RGB_D_Set------------
// Set new statuses for the RGB LEDs.
// non-zero is fully on.
// 0 is fully off.
// Input: red is status for red
//        green is status for green
//        blue is status for blue
// Output: none
// Assumes: BSP_RGB_D_Init() has been called
void BSP_RGB_D_Set(int red, int green, int blue){
  if(red){
    RED = 0x08;
  } else{
    RED = 0x00;
  }
  if(green){
    GREEN = 0x08;
  } else{
    GREEN = 0x00;
  }
  if(blue){
    BLUE = 0x10;
  } else{
    BLUE = 0x00;
  }
}
void BSP_RGB_D_Init(int red, int green, int blue){
  SYSCTL_RCGCGPIO_R |= 0x00000026; // 1) activate clock for Ports F, C, and B
  while((SYSCTL_PRGPIO_R&0x26) != 0x26){};// allow time for clocks to stabilize
                                   // 2) no need to unlock PF3, PC4, or PB3
  GPIO_PORTF_AMSEL_R &= ~0x08;     // 3a) disable analog on PF3
  GPIO_PORTC_AMSEL_R &= ~0x10;     // 3b) disable analog on PC4
  GPIO_PORTB_AMSEL_R &= ~0x08;     // 3c) disable analog on PB3
                                   // 4a) configure PF3 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF0FFF)+0x00000000;
                                   // 4b) configure PC4 as GPIO
  GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xFFF0FFFF)+0x00000000;
                                   // 4c) configure PB3 as GPIO
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF0FFF)+0x00000000;
  GPIO_PORTF_DIR_R |= 0x08;        // 5a) make PF3 output
  GPIO_PORTC_DIR_R |= 0x10;        // 5b) make PC4 output
  GPIO_PORTB_DIR_R |= 0x08;        // 5c) make PB3 output
  GPIO_PORTF_AFSEL_R &= ~0x08;     // 6a) disable alt funct on PF3
  GPIO_PORTC_AFSEL_R &= ~0x10;     // 6b) disable alt funct on PC4
  GPIO_PORTB_AFSEL_R &= ~0x08;     // 6c) disable alt funct on PB3
  GPIO_PORTF_PUR_R &= ~0x08;       // disable pull-up on PF3
  GPIO_PORTC_PUR_R &= ~0x10;       // disable pull-up on PC4
  GPIO_PORTB_PUR_R &= ~0x08;       // disable pull-up on PB3
  GPIO_PORTF_DEN_R |= 0x08;        // 7a) enable digital I/O on PF3
  GPIO_PORTC_DEN_R |= 0x10;        // 7b) enable digital I/O on PC4
  GPIO_PORTB_DEN_R |= 0x08;        // 7c) enable digital I/O on PB3
  BSP_RGB_D_Set(red, green, blue);
}
// ------------BSP_RGB_D_Toggle------------
// Toggle the statuses of the RGB LEDs.
// non-zero is toggle.
// 0 is do not toggle.
// Input: red is toggle for red
//        green is toggle for green
//        blue is toggle for blue
// Output: none
// Assumes: BSP_RGB_D_Init() has been called
void BSP_RGB_D_Toggle(int red, int green, int blue){
  if(red){
    RED = RED^0x08;
  }
  if(green){
    GREEN = GREEN^0x08;
  }
  if(blue){
    BLUE = BLUE^0x10;
  }
}
   
void BSP_UART0_Init()
   {// Configuring uart0
   SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // activate UART0
   SYSCTL_RCGCGPIO_R |= 0x01; // activate port A
   UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
   //UART0_IBRD_R = 43;                    // IBRD = int(80,000,000 / (16 * 115200)) = int(43.402778)
   //UART0_FBRD_R = 26;                    // FBRD = round(0.402778 * 64) = 26
   UART0_IBRD_R = 27;                    // IBRD = int(50,000,000 / (16 * 115200)) = int(27.1267361)
   UART0_FBRD_R = 8;                    // FBRD = round(.1267361 * 64) = 8
                                       // 8 bit word length (no parity bits, one stop bit, FIFOs)
   UART0_LCRH_R = (UART_LCRH_WLEN_8);//|UART_LCRH_FEN);
   UART0_CTL_R |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;       // enable UART  TX, RX
   GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
   GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
                                        // configure PA1-0 as UART
   GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
   GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
   UART0_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
	NVIC_EN0_R = 1<<5;
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    for (unsigned int i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}
void Uart0Isr()
{
	char c = UART0_DR_R;
	putcUart0(c);
}



static void BSP_i2cinit(void){
  SYSCTL_RCGCI2C_R |= 0x0002;      // 1a) activate clock for I2C1
  SYSCTL_RCGCGPIO_R |= 0x0001;     // 1b) activate clock for Port A
  while((SYSCTL_PRGPIO_R&0x01) == 0){};// allow time for clock to stabilize
                                   // 2) no need to unlock PA7-6
  GPIO_PORTA_AMSEL_R &= ~0xC0;     // 3) disable analog functionality on PA7-6
                                   // 4) configure PA7-6 as I2C1
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0x00FFFFFF)+0x33000000;
  GPIO_PORTA_ODR_R |= 0x80;        // 5) enable open drain on PA7 only
  GPIO_PORTA_AFSEL_R |= 0xC0;      // 6) enable alt funct on PA7-6
  GPIO_PORTA_DEN_R |= 0xC0;        // 7) enable digital I/O on PA7-6
  I2C1_MCR_R = I2C_MCR_MFE;        // 8) master function enable
  I2C1_MTPR_R = 39;                // 9) configure for 100 kbps clock
  // 20*(TPR+1)*12.5ns = 10us, with TPR=39
}

static void BSP_i2c1_init(void){
  BSP_i2cinit();                   // 1) activate clock for Port A (done in BSP_i2cinit())
                                   // allow time for clock to stabilize (done in BSP_i2cinit())
                                   // 2) no need to unlock PA2
  GPIO_PORTA_AMSEL_R &= ~0x20;     // 3) disable analog on PA5
                                   // 4) configure PA5 as GPIO
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0FFFFF)+0x00000000;
  GPIO_PORTA_DIR_R &= ~0x20;       // 5) make PA5 input
  GPIO_PORTA_AFSEL_R &= ~0x20;     // 6) disable alt funct on PA5
  GPIO_PORTA_DEN_R |= 0x20;        // 7) enable digital I/O on PA5
}

#if (0)
static void BSP_i2c2_init(void){
  BSP_i2cinit();                   // 1) activate clock for Port A (done in BSP_i2cinit())
                                   // allow time for clock to stabilize (done in BSP_i2cinit())
                                   // 2) no need to unlock PA2
  GPIO_PORTA_AMSEL_R &= ~0x04;     // 3) disable analog on PA2
                                   // 4) configure PA2 as GPIO
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTA_DIR_R &= ~0x04;       // 5) make PA5 input
  GPIO_PORTA_AFSEL_R &= ~0x04;     // 6) disable alt funct on PA2
  GPIO_PORTA_DEN_R |= 0x04;        // 7) enable digital I/O on PA2
}

// receives two bytes from specified slave
// Note for HMC6352 compass only:
// Used with 'A' commands
// Note for TMP102 thermometer only:
// Used to read the contents of the pointer register
static uint16_t I2C_Recv2(int8_t slave){
  uint8_t data1,data2;
  int retryCounter = 1;
  do{
    while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
    I2C1_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
    I2C1_MSA_R |= 0x01;              // MSA[0] is 1 for receive
    I2C1_MCS_R = (0
                         | I2C_MCS_ACK      // positive data ack
//                         & ~I2C_MCS_STOP    // no stop
                         | I2C_MCS_START    // generate start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data1 = (I2C1_MDR_R&0xFF);       // MSB data sent first
    I2C1_MCS_R = (0
//                         & ~I2C_MCS_ACK     // negative data ack (last byte)
                         | I2C_MCS_STOP     // generate stop
//                         & ~I2C_MCS_START   // no start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data2 = (I2C1_MDR_R&0xFF);       // LSB data sent last
    retryCounter = retryCounter + 1;        // increment retry counter
  }                                         // repeat if error
  while(((I2C1_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
  return (data1<<8)+data2;                  // usually returns 0xFFFF on error
}

// sends three bytes to specified slave
// Note for HMC6352 compass only:
// Used with 'w' and 'G' commands
// Note for TMP102 thermometer only:
// Used to change the contents of the pointer register
// Returns 0 if successful, nonzero if error
static uint16_t I2C_Send3(int8_t slave, uint8_t data1, uint8_t data2, uint8_t data3){
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C1_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C1_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C1_MDR_R = data1&0xFF;         // prepare first byte
  I2C1_MCS_R = (0
//                       & ~I2C_MCS_ACK     // no data ack (no data on send)
//                       & ~I2C_MCS_STOP    // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C1_MCS_R = (0                // send stop if nonzero
//                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
//                       & ~I2C_MCS_START   // no start/restart
//                       & ~I2C_MCS_RUN     // master disable
                       );
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C1_MDR_R = data2&0xFF;         // prepare second byte
  I2C1_MCS_R = (0
//                       & ~I2C_MCS_ACK     // no data ack (no data on send)
//                       & ~I2C_MCS_STOP    // no stop
//                       & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C1_MCS_R = (0                // send stop if nonzero
//                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
//                       & ~I2C_MCS_START   // no start/restart
//                       & ~I2C_MCS_RUN   // master disable
                        );
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C1_MDR_R = data3&0xFF;         // prepare third byte
  I2C1_MCS_R = (0
//                       & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
//                       & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}
#endif

static void prvTask_producer(void * pvParameters)
{
    //TickType_t xNextWakeTime;
    //const TickType_t xBlockTime = mainTASK_SEND_FREQUENCY_MS;
    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    uint16_t x_val, y_val;
    uint8_t select_val;
    for( ; ; )
    {
        //vTaskDelayUntil( &xNextWakeTime, xBlockTime );
        putsUart0( "Sending value \r\n");
        BSP_RGB_D_Toggle(1,1,1);
        BSP_Joystick_Input(&x_val, &y_val, &select_val);
        xQueueSend( xQueue, &x_val, 0U );
        xQueueSend( yQueue, &y_val, 0U );
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
}
static void prvTask_consumer(void * pvParameters)
{
    /* Prevent the compiler warning about the unused parameter. */
    ( void ) pvParameters;
    char x_buffer[5];
    char y_buffer[5];
    uint16_t x_val, y_val;
    //uint8_t select_val;
    for( ; ; )
    {
        // xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );
        xQueueReceive( xQueue, &x_val, portMAX_DELAY );
        xQueueReceive( yQueue, &y_val, portMAX_DELAY );
        putsUart0( "Received value \r\n");
        itoa((int32_t)(x_val), x_buffer, 10);
        itoa((int32_t)(y_val), y_buffer, 10);
        putsUart0( "X value: ");
        putsUart0(x_buffer);
        putsUart0( " Y value: ");
        putsUart0(y_buffer);
        putsUart0( "\r\n");
        vTaskDelay( pdMS_TO_TICKS( 200 ) ); 

    }
}

int main(void){
  //BSP_init();
  //DisableInterrupts();
  IntMasterDisable();
  BSP_SysCtl_mcuRev();
  //BSP_Clock_InitFastest();
  BSP_Clock_Init_50Mz();
  SysTick_Init();               // initialize SysTick timer
  BSP_UART0_Init();
  putsUart0("\r\n-------------- WELCOME ----------------\r\n");
  putsUart0("Initiating UART0 Done.......\r\n");
  BSP_Button1_Init();
  putsUart0("Initiating Button1 Done.......\r\n");
  BSP_Button2_Init();
  putsUart0("Initiating Button2 Done.......\r\n");
  BSP_Joystick_Init();
  putsUart0("Initiating Joystick Done.......\r\n");
  BSP_RGB_D_Init(1, 1, 1);
  putsUart0("Initiating Booster LEDs Done.......\r\n");
  //BSP_RGB_onboard_Init(1, 1, 1);
  putsUart0("Initiating LEDs Done.......\r\n");
  BSP_i2c1_init();
  putsUart0("Initiating I2C Done.......\r\n");
  //BSP_LCD_Init();
  //putsUart0("Initiating LCD Done.......\r\n");
  //BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
  //putsUart0("\r\n");
  //EnableInterrupts();
  IntMasterEnable();
  //BSP_Delay1ms(100);
  /* Initialize and start a watchdog timer: */
  //if ( 0 == watchdogInit( 0, 10000) )
  //{
  //    putsUart0("Initialization of watchdog reloading failed\r\n");
  //}
  //SysTick80_Wait10ms(1);  // approximately 10 ms

  xQueue = xQueueCreate(3, sizeof(uint16_t));
  yQueue = xQueueCreate(3, sizeof(uint16_t));
  putsUart0("System Init Done.......\r\n");

  /*---------------------Task Creation Block----------------------*/
  xTaskCreate( prvTask_producer,             /* The function that implements the task. */
          "ProduceTask",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
          configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
          NULL,                            /* The parameter passed to the task - not used in this simple case. */
          mainTASK_A_PRIORITY, /* The priority assigned to the task. */
          NULL );                          /* The task handle is not required, so NULL is passed. */    

  xTaskCreate( prvTask_consumer,             /* The function that implements the task. */
          "ConsumeTask",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
          configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
          NULL,                            /* The parameter passed to the task - not used in this simple case. */
          mainTASK_B_PRIORITY, /* The priority assigned to the task. */
          NULL );                          /* The task handle is not required, so NULL is passed. */    

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler();
  
  putsUart0("Could not start the scheduler!!!\r\n");
  while(1){};

  return 0;
}
