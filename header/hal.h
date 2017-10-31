/*
 * hal_init.h
 *
 *  Created on: Aug 25, 2016
 *      Author: Venugopal Velcheru
 */
//-----------------------------------------------------------------------------
// Hardware Abstraction Layer Initialization
//-----------------------------------------------------------------------------
#ifndef PUBLIC_HAL_H_
#define PUBLIC_HAL_H_
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
typedef uint8_t BPACK_data_t;

#define DELAY_1uSEC		1
#define DELAY_1mSEC		1000
#define DELAY_500mSEC   500000
#define DELAY_1SEC		1000000
#define DELAY_10SEC		10000000

#define HAL_LED_ON     1
#define HAL_LED_OFF	   0
#define HAL_LED_TOGGLE 1
//-----------------------------------------------------------------------------
// GPIO pin access
//-----------------------------------------------------------------------------
#define HAL_PORT_RESET      0x00
#define HAL_PORT_ENABLE     0xFF
#define HAL_GPIO_BIT0		0x01
#define HAL_GPIO_BIT1		0x02
#define HAL_GPIO_BIT2		0x04
#define HAL_GPIO_BIT3		0x08
#define HAL_GPIO_BIT4		0x10
#define HAL_GPIO_BIT5		0x20
#define HAL_GPIO_BIT6		0x40
#define HAL_GPIO_BIT7		0x80
//-----------------------------------------------------------------------------
// PWM configuration
//-----------------------------------------------------------------------------
#define HAL_PWM_BUZZ_LOAD  (40000000/(50*16))// LOAD = PWMclk/ReqFreq, PWMclk = SYSclk/16
#define HAL_PWM_DUTY50 0.5
#define HAL_PWM_DUTY25 0.75
#define HAL_PWM_DUTY75 0.25
#define HAL_PWM_BUZZ_CMP    ((HAL_PWM_BUZZ_LOAD +1) * HAL_PWM_DUTY50)-1

#define HAL_BPAC_BUZZ_ON      (PWM1_ENABLE_R |= PWM_ENABLE_PWM6EN)
#define HAL_BPAC_BUZZ_OFF     (PWM1_ENABLE_R &= ~PWM_ENABLE_PWM6EN)
//-----------------------------------------------------------------------------
// Switches
//-----------------------------------------------------------------------------
#define HAL_LPAD_SW1_STATUS ((GPIO_PORTF_DATA_R & 0x10)>>4)
#define HAL_LPAD_SW2_STATUS (GPIO_PORTF_DATA_R & 0x01)
#define HAL_BPAC_SW1_STATUS ((GPIO_PORTD_DATA_R & 0x40)>>6)
#define HAL_BPAC_SW2_STATUS ((GPIO_PORTD_DATA_R & 0x80)>>7)
#define HAL_BPAC_SWJ_STATUS (*((volatile uint32_t *)0x40024040))  /* PE4 */
//-----------------------------------------------------------------------------
// LEDs
//-----------------------------------------------------------------------------
#define HAL_LPAD_RED_LED_ON   GPIO_PORTF_DATA_R |= HAL_GPIO_BIT1
#define HAL_LPAD_RED_LED_OFF  GPIO_PORTF_DATA_R &= ~HAL_GPIO_BIT1
#define HAL_BPAC_GRN_LED_ON   GPIO_PORTB_DATA_R |= HAL_GPIO_BIT3
#define HAL_BPAC_GRN_LED_OFF  GPIO_PORTB_DATA_R &= ~HAL_GPIO_BIT3
#define HAL_BPAC_BLU_LED_ON   (PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN)
#define HAL_BPAC_BLU_LED_OFF  (PWM0_ENABLE_R &= ~PWM_ENABLE_PWM6EN)
//-----------------------------------------------------------------------------
// Accelerometer Configuration
//-----------------------------------------------------------------------------
#define HAL_BPAC_ACC_AXIS_X_AIN   7     // BPAC Accelerometer x-axis uses AIN7
#define HAL_BPAC_ACC_AXIS_Y_AIN   6     // BPAC Accelerometer y-axis uses AIN6
#define HAL_BPAC_ACC_AXIS_Z_AIN   5     // BPAC Accelerometer z-axis uses AIN5
//-----------------------------------------------------------------------------
// Joystick Configuration
//-----------------------------------------------------------------------------
#define HAL_BPAC_JOY_AXIS_X_AIN   11    // BPAC JOYSTICK x-axis uses AIN11
#define HAL_BPAC_JOY_AXIS_Y_AIN   4     // BPAC JOYSTICK y-axis uses AIN4

#define HAL_LCD_CS     	(*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))

//#define RED_LED     	(*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
//#define BLUE_LED    	(*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
//#define GREEN_LED   	(*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
//#define YELLOW_LED  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

//-----------------------------------------------------------------------------
// Device prototype, function and method decleration
//-----------------------------------------------------------------------------
void HAL_Reset_ISR(void);
void HAL_Sys_Delay(uint32_t us);
void HAL_Sys_Clk_Init(void);
void HAL_Button1_Init(void);
void HAL_Button2_Init(void);
void HAL_Accelero_Init(void);
void HAL_RGB_BPACK_Init(void);
void HAL_Buzzer_Init(void);
void HAL_Microphone_Init();
void HAL_Joystick_Init(void);
void HAL_RGB_LPAD_Init(void);
void HAL_ButtonLPAD_Init(void);
void HAL_BT_UART_Init(void);
void HAL_LPAD_UART_Init(void);

bool HAL_Button1_Input(void);
bool HAL_Button2_Input(void);
void HAL_Accelero_Input(void);
void HAL_RGB_BPACK_Set(uint8_t, uint8_t, uint8_t);
void HAL_Buzzer_Set(uint8_t, uint8_t);
void HAL_Microphone_Input(void);
void HAL_Joystick_Input(uint16_t*, uint16_t*, uint32_t*);
void HAL_RGB_LPAD_Set(uint8_t, uint8_t, uint8_t);
void HAL_Application_Start();
void HAL_LPAD_UART_Write(uint8_t);
void HAL_Init(void);
//int16_t hal_ADC0_readSs3();

#endif /* PUBLIC_HAL_INIT_H_ */
