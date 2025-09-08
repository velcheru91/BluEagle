#include <stdint.h>
#include "BSP_delay.h"
// delay function from sysctl.c
// which delays 3.3*ulCount cycles
// ulCount=23746 => 1ms = 23746*3.3cycle/loop/80,000
void parrotdelay(uint32_t ulCount){
  __asm (  "    subs    r0, #1\n"
      "    bne     Delay\n"
      "    bx      lr\n");
}
// ------------BSP_Delay1ms------------
// Simple delay function which delays about n
// milliseconds.
// Inputs: n  number of 1 msec to wait
// Outputs: none
void BSP_Delay1ms(uint32_t n){
  while(n){
    parrotdelay(23746);    // 1 msec, tuned at 80 MHz, originally part of LCD module
    n--;
  }
}
