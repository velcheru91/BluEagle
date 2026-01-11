// Runs on either the TM4C123 or MSP432 with an Educational BoosterPack MKII (BOOSTXL-EDUMKII)
// This file contains the function prototypes for the software interface to the MKII BoosterPack.
// This board support package (BSP) is an abstraction layer,
//   forming a bridge between the low-level hardware and the high-level software.

// Daniel and Jonathan Valvano
// June 8, 2016

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2016

   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2016

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

// Connected pins in physical order
// J1.1 +3.3V (power)
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5/AIN11, MSP432 P6.0}
// J1.3 UART from Bluetooth to LaunchPad (UART) {TM4C123 PB0, MSP432 P3.2}
// J1.4 UART from LaunchPad to Bluetooth (UART) {TM4C123 PB1, MSP432 P3.3}
// J1.5 joystick Select button (digital) {TM4C123 PE4, MSP432 P4.1}
// J1.6 microphone (analog)              {TM4C123 PE5/AIN8, MSP432 P4.3}
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4, MSP432 P1.5}
// J1.8 ambient light (OPT3001) interrupt (digital) {TM4C123 PA5, MSP432 P4.6}
// J1.9 ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL (I2C)  {TM4C123 PA6, MSP432 P6.5}
// J1.10 ambient light (OPT3001) and temperature sensor (TMP006) I2C SDA (I2C) {TM4C123 PA7, MSP432 P6.4}
//--------------------------------------------------
// J2.11 temperature sensor (TMP006) interrupt (digital) {TM4C123 PA2, MSP432 P3.6}
// J2.12 nothing                         {TM4C123 PA3, MSP432 P5.2}
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4, MSP432 P5.0}
// J2.14 nothing                         {TM4C123 PB6, MSP432 P1.7}
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7, MSP432 P1.6}
// J2.16 nothing (reset)
// J2.17 LCD !RST (digital)              {TM4C123 PF0, MSP432 P5.7}
// J2.18 Profile 4                       {TM4C123 PE0, MSP432 P3.0}
// J2.19 servo PWM                       {TM4C123 PB2, MSP432 P2.5}
// J2.20 GND (ground)
//--------------------------------------------------
// J3.21 +5V (power)
// J3.22 GND (ground)
// J3.23 accelerometer X (analog)        {TM4C123 PD0/AIN7, MSP432 P6.1}
// J3.24 accelerometer Y (analog)        {TM4C123 PD1/AIN6, MSP432 P4.0}
// J3.25 accelerometer Z (analog)        {TM4C123 PD2/AIN5, MSP432 P4.2}
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3/AIN4, MSP432 P4.4}
// J3.27 Profile 0                       {TM4C123 PE1, MSP432 P4.5}
// J3.28 Profile 1                       {TM4C123 PE2, MSP432 P4.7}
// J3.29 Profile 2                       {TM4C123 PE3, MSP432 P5.4}
// J3.30 Profile 3                       {TM4C123 PF1, MSP432 P5.5}
//--------------------------------------------------
// J4.31 LCD RS (digital)                {TM4C123 PF4, MSP432 P3.7}
// J4.32 user Button2 (bottom) (digital) {TM4C123 PD7, MSP432 P3.5}
// J4.33 user Button1 (top) (digital)    {TM4C123 PD6, MSP432 P5,1}
// J4.34 Profile 6/gator hole switch     {TM4C123 PC7, MSP432 P2.3}
// J4.35 nothing                         {TM4C123 PC6, MSP432 P6.7}
// J4.36 Profile 5                       {TM4C123 PC5, MSP432 P6.6}
// J4.37 RGB LED blue (PWM)              {TM4C123 PC4, MSP432 P5.6}
// J4.38 RGB LED green (PWM)             {TM4C123 PB3, MSP432 P2.4}
// J4.39 RGB LED red (jumper up) or LCD backlight (jumper down) (PWM) {TM4C123 PF3, MSP432 P2.6}
// J4.40 buzzer (PWM)                    {TM4C123 PF2, MSP432 P2.7}
//--------------------------------------------------
// Connected pins in logic order
// power and reset
// J1.1 +3.3V (power)
// J3.21 +5V (power)
// J3.22 GND (ground)
// J2.20 GND (ground)
// J2.16 nothing (reset)
//--------------------------------------------------
// LCD graphics
// J1.7 LCD SPI clock (SPI)              {TM4C123 PB4, MSP432 P1.5}
// J2.13 LCD SPI CS (SPI)                {TM4C123 PA4, MSP432 P5.0}
// J2.15 LCD SPI data (SPI)              {TM4C123 PB7, MSP432 P1.6}
// J2.17 LCD !RST (digital)              {TM4C123 PF0, MSP432 P5.7}
// J4.31 LCD RS (digital)                {TM4C123 PF4, MSP432 P3.7}
//--------------------------------------------------
// 3-color LED
// J4.37 RGB LED blue (PWM)              {TM4C123 PC4, MSP432 P5.6}
// J4.38 RGB LED green (PWM)             {TM4C123 PB3, MSP432 P2.4}
// J4.39 RGB LED red (jumper up) or LCD backlight (jumper down) (PWM) {TM4C123 PF3, MSP432 P2.6}
//--------------------------------------------------
// user buttons
// J4.32 user Button2 (bottom) (digital) {TM4C123 PD7, MSP432 P3.5}
// J4.33 user Button1 (top) (digital)    {TM4C123 PD6, MSP432 P5.1}
//--------------------------------------------------
// buzzer output
// J4.40 buzzer (PWM)                    {TM4C123 PF2, MSP432 P2.7}
//--------------------------------------------------
// Joystick
// J1.5 joystick Select button (digital) {TM4C123 PE4, MSP432 P4.1}
// J1.2 joystick horizontal (X) (analog) {TM4C123 PB5/AIN11, MSP432 P6.0}
// J3.26 joystick vertical (Y) (analog)  {TM4C123 PD3/AIN4, MSP432 P4.4}
//--------------------------------------------------
// accelerometer
// J3.23 accelerometer X (analog)        {TM4C123 PD0/AIN7, MSP432 P6.1}
// J3.24 accelerometer Y (analog)        {TM4C123 PD1/AIN6, MSP432 P4.0}
// J3.25 accelerometer Z (analog)        {TM4C123 PD2/AIN5, MSP432 P4.2}
//--------------------------------------------------
// microphone
// J1.6 microphone (analog)              {TM4C123 PE5/AIN8, MSP432 P4.3}
//--------------------------------------------------
// light and temperature sensors (I2C)
// J1.8 ambient light (OPT3001) interrupt (digital) {TM4C123 PA5, MSP432 P4.6}
// J1.9 ambient light (OPT3001) and temperature sensor (TMP006) I2C SCL (I2C)  {TM4C123 PA6, MSP432 P6.5}
// J1.10 ambient light (OPT3001) and temperature sensor (TMP006) I2C SDA (I2C) {TM4C123 PA7, MSP432 P6.4}
// J2.11 temperature sensor (TMP006) interrupt (digital) {TM4C123 PA2, MSP432 P3.6}
//--------------------------------------------------
// Bluetooth booster
// J1.3 UART from Bluetooth to LaunchPad (UART) {TM4C123 PB0, MSP432 P3.2}
// J1.4 UART from LaunchPad to Bluetooth (UART) {TM4C123 PB1, MSP432 P3.3}
//--------------------------------------------------
// profile pins
// J3.27 Profile 0                       {TM4C123 PE1, MSP432 P4.5}
// J3.28 Profile 1                       {TM4C123 PE2, MSP432 P4.7}
// J3.29 Profile 2                       {TM4C123 PE3, MSP432 P5.4}
// J3.30 Profile 3                       {TM4C123 PF1, MSP432 P5.5}
// J2.18 Profile 4                       {TM4C123 PE0, MSP432 P3.0}
// J4.36 Profile 5                       {TM4C123 PC5, MSP432 P6.6}
// J4.34 Profile 6                       {TM4C123 PC7, MSP432 P2.3}
//--------------------------------------------------
// unconnected pins
// J2.12 nothing                         {TM4C123 PA3, MSP432 P5.2}
// J2.14 nothing                         {TM4C123 PB6, MSP432 P1.7}
// J2.19 servo PWM                       {TM4C123 PB2, MSP432 P2.5}
// J4.35 nothing                         {TM4C123 PC6, MSP432 P6.7}
#include <stdint.h>
#include "BSP.h"
#include "tm4c123gh6pm.h"

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
void SysTick_Delay(uint32_t delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// delay function from sysctl.c
// which delays 3.3*ulCount cycles
// ulCount=23746 => 1ms = 23746*3.3cycle/loop/80,000
void parrotdelay(uint32_t ulCount){
  if(ulCount == 0){}
  __asm (  "    subs    r0, #1\n"
      "    bne     parrotdelay\n"
      "    bx      lr\n");
}
// ------------BSP_Delay1ms------------
// Simple delay function which delays about n
// milliseconds.
// Inputs: n  number of 1 msec to wait
// Outputs: none
void BSP_Delay1ms(uint32_t n){
  uint32_t i;
  for(i=0; i<n; i++){
    SysTick_Delay(500000);  // wait 1ms (assumes 50MHz clock)
  }
  //while(n){
    //parrotdelay(23746);    // 1 msec, tuned at 80 MHz, originally part of LCD module
  //  parrotdelay(14836);    // 1 msec, tuned at 50 MHz
  //  n--;
  //}
}
