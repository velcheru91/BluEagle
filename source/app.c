/*
 * app.c
 *
 *  Created on: Oct 31, 2017
 *      Author: Venugopal Velcheru
 */

//-----------------------------------------------------------------------------
// Application Layer Task Management File
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
#ifndef BLUEAGLE_HEADER_APP_H_
#include <app.h>
#endif
//-----------------------------------------------------------------------------
// Declarations
//-----------------------------------------------------------------------------
struct USER_INPUT{
    uint16_t X_axis;
    uint16_t Y_axis;
    uint32_t button;
} Joypad;

struct STEPPER_DATA{
    uint16_t BaseData;
    uint16_t LeftData;
    uint16_t RightData;
    uint16_t FingerData;
} motor;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void app_Incipient(void)
{
    HAL_Init();
}

void app_Task_Read_Stick(void)
{
    HAL_Joystick_Input(&Joypad.X_axis, &Joypad.Y_axis, &Joypad.button);
}

void app_Task_Set_Motor1(void)
{

    if ((Joypad.X_axis > 760) && (TM1.count > TM1.load))
    {
        TM1.count = 0;
        if (motor.RightData < (500*89))
            motor.RightData = motor.RightData + 5;
    }

    if ((Joypad.X_axis < 256) && (TM1.count > TM1.load))
    {
        TM1.count = 0;
        if (motor.RightData > (500*85))
            motor.RightData = motor.RightData - 5;
    }
    PWM1_3_CMPA_R = motor.RightData;
}

void app_Task_Set_Motor2(void)
{

    if ((Joypad.Y_axis < 256) && (TM2.count > TM2.load))
    {
        TM2.count = 0;
        if (motor.LeftData < ((49999*94)/100))
            motor.LeftData = motor.LeftData + 6;
    }

    if ((Joypad.Y_axis > 760) && (TM2.count > TM2.load))
    {
        TM2.count = 0;
        if (motor.LeftData > ((49999*89)/100))
            motor.LeftData = motor.LeftData - 6;
    }
    PWM0_3_CMPA_R = motor.LeftData;
}

void app_Reprise (void)
{
//    uint32_t* sel_pt;
//    uint16_t raw_x, raw_y;
//  uint8_t out_x, out_y;
//    uint8_t dir1=1,dir2=1;
//    uint32_t j=850,k=880, data1=0, data2=0;

/*    PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90); // reset to position 0
    PWM1_ENABLE_R |= PWM_ENABLE_PWM6EN;
    motor.RightData = ((HAL_PWM_BUZZ_LOAD-1)*0.90);*/

//  PWM1_1_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);
//  PWM1_ENABLE_R |= PWM_ENABLE_PWM3EN;

    PWM0_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);
    PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN;
    motor.LeftData = ((HAL_PWM_BUZZ_LOAD-1)*0.90);

//    PWM0_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);
//    PWM0_ENABLE_R |= PWM_ENABLE_PWM7EN;
    while(1)
    {
        app_Task_Read_Stick();
 //       app_Task_Set_Motor1();
        app_Task_Set_Motor2();

    }
        //HAL_Joystick_Input(&raw_x, &raw_y, sel_pt);
//        if ((GPIO_PORTF_DATA_R & 0x01) == 0)


//        if ((GPIO_PORTF_DATA_R & 0x10) == 0)

//  }




//  while (1){

//      scale_value_x = ((((double)(raw_x)-1000.0)*256.0)/4096.0);
//      out_x = (uint8_t)floor(scale_value_x);
//      scale_value_y = ((((double)(raw_y)-1000.0)*256.0)/4096.0);
//      out_y = (uint8_t)floor(scale_value_y);
        //HAL_LPAD_UART_Write(out_x);
        //HAL_LPAD_UART_Write(out_y);
        //HAL_LPAD_UART_Write(0x0A);
//      if(HAL_Button1_Input())
//      {
//          HAL_RGB_BPACK_Set(0,1,0);
//          HAL_Buzzer_Set(1,0);
//      }
//      else if(HAL_Button2_Input())
//      {
//          HAL_RGB_BPACK_Set(0,0,1);
//          HAL_Buzzer_Set(1,1);
//      }
//      else if(!(((*sel_pt) & HAL_GPIO_BIT4)>>4))
//      {
//          HAL_RGB_BPACK_Set(0,0,0);
//          HAL_Buzzer_Set(1,2);
//      }
//      else
//      {
//          HAL_Buzzer_Set(0,0);
//      }
//      uint8_t i,j;
//      for (i=0;i<8;i++)
//      {
//          HAL_Buzzer_Set(0,0);
//          HAL_Buzzer_Set(1,i);
//          HAL_Sys_Delay(DELAY_1mSEC*500);
//      }
//      for (j=8;j>0;j--)
//      {
//          HAL_Buzzer_Set(0,0);
//          HAL_Buzzer_Set(1,j-1);
        //if(HAL_Button1_Input())

//      HAL_Sys_Delay(DELAY_1SEC);
//      {
//          PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.950);
//          HAL_BPAC_BUZZ_ON
//      }
//      HAL_Sys_Delay(DELAY_1SEC);
        //else if (HAL_Button2_Input())
//      {
//          PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.925);
//      }
//      HAL_Sys_Delay(DELAY_1SEC);
        //else
//      {
//          PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.900);
//      }

//      HAL_Sys_Delay(DELAY_1SEC);
//
//      HAL_Sys_Delay(DELAY_1SEC);
//
//      HAL_Sys_Delay(DELAY_1SEC);
//      HAL_Buzzer_Set(1,0);
//      HAL_Sys_Delay(DELAY_1SEC);
//      }
//  }
        //raw_input = hal_ADC0_readSs3();
        //scaled_value = ((((double)raw_input-1000.0)*256.0)/2000.0);
        //acc_xaxis_output = (uint8_t)floor(scaled_value);
        //while (UART0_FR_R & UART_FR_TXFF);
        //UART0_DR_R = (uint8_t)acc_xaxis_output;
        //while (UART1_FR_R & UART_FR_TXFF);
        //UART1_DR_R = (uint8_t)acc_xaxis_output;
}
//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------

