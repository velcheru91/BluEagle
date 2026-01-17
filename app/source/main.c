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
#include "driver_pca9685.h"
#include "driver_pca9685_interface.h"

//#include "wdtask.h"
#define mainTASK_A_PRIORITY                 (tskIDLE_PRIORITY + 1)
#define mainTASK_B_PRIORITY                 (tskIDLE_PRIORITY + 4)
#define MAXRETRIES              5  // number of receive attempts before giving up

QueueHandle_t xQueue = NULL;
QueueHandle_t yQueue = NULL;
static pca9685_handle_t gs_handle;        /**< pca9685 handle */

//void DisableInterrupts(void); // Disable interrupts
//void EnableInterrupts(void);  // Enable interrupts
//long StartCritical (void);    // previous I bit, disable interrupts
//void EndCritical(long sr);    // restore I bit to previous value
//void WaitForInterrupt(void);  // low power mode


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
        //BSP_RGB_D_Toggle(0,1,1);
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
        BSP_LCD_SetCursor(5, 3);
        BSP_LCD_OutUDec4((uint32_t)x_val, BSP_LCD_Color565(255, 255, 255));
        BSP_LCD_SetCursor(5, 4);
        BSP_LCD_OutUDec4((uint32_t)y_val, BSP_LCD_Color565(255, 255, 255));
        vTaskDelay( pdMS_TO_TICKS( 200 ) ); 

    }
}

uint8_t pca9685_servo_init(pca9685_address_t addr, pca9685_channel_t channel, uint32_t times)
{
    uint8_t res;
    //uint8_t reg;
    //uint16_t on_count, off_count;
    //uint32_t i;
    pca9685_info_t info;
    
    /* link interface function */
    DRIVER_PCA9685_LINK_INIT(&gs_handle, pca9685_handle_t);
    DRIVER_PCA9685_LINK_IIC_INIT(&gs_handle, pca9685_interface_iic_init);
    DRIVER_PCA9685_LINK_IIC_DEINIT(&gs_handle, pca9685_interface_iic_deinit);
    DRIVER_PCA9685_LINK_IIC_READ(&gs_handle, pca9685_interface_iic_read);
    DRIVER_PCA9685_LINK_IIC_WEITE(&gs_handle, pca9685_interface_iic_write);
    DRIVER_PCA9685_LINK_OE_GPIO_INIT(&gs_handle, pca9685_interface_oe_init);
    DRIVER_PCA9685_LINK_OE_GPIO_DEINIT(&gs_handle, pca9685_interface_oe_deinit);
    DRIVER_PCA9685_LINK_OE_GPIO_WRITE(&gs_handle, pca9685_interface_oe_write);
    DRIVER_PCA9685_LINK_DELAY_MS(&gs_handle, pca9685_interface_delay_ms);
    DRIVER_PCA9685_LINK_DEBUG_PRINT(&gs_handle, pca9685_interface_debug_print);
    
    /* get information */
    res = pca9685_info(&info);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: get info failed.\n");
        
        return 1;
    }
    else
    {
        /* print chip info */
        pca9685_interface_debug_print("pca9685: chip is %s.\n", info.chip_name);
        pca9685_interface_debug_print("pca9685: manufacturer is %s.\n", info.manufacturer_name);
        pca9685_interface_debug_print("pca9685: interface is %s.\n", info.interface);
        pca9685_interface_debug_print("pca9685: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        pca9685_interface_debug_print("pca9685: min supply voltage is %d V.\n", info.supply_voltage_min_v);
        pca9685_interface_debug_print("pca9685: max supply voltage is %d V.\n", info.supply_voltage_max_v);
        pca9685_interface_debug_print("pca9685: max current is %d mA.\n", info.max_current_ma);
        pca9685_interface_debug_print("pca9685: max temperature is %d C.\n", info.temperature_max);
        pca9685_interface_debug_print("pca9685: min temperature is %d C.\n", info.temperature_min);
    }
    
    /* start write test */
    pca9685_interface_debug_print("pca9685: start write test.\n");
    
    /* set addr pin */
    res = pca9685_set_addr_pin(&gs_handle, addr);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set addr pin failed.\n");
        
        return 1;
    }
    
    /* pca9685 init */
    res = pca9685_init(&gs_handle);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: init failed.\n");
        
        return 1;
    }
    
    /* inactive */
    res = pca9685_set_active(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set active failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set sleep mode */
    res = pca9685_set_sleep_mode(&gs_handle, PCA9685_BOOL_TRUE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set sleep mode failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set 50Hz */
    //res = pca9685_output_frequency_convert_to_register(&gs_handle, PCA9685_OSCILLATOR_INTERNAL_FREQUENCY, 50, (uint8_t *)&reg);
    //if (res != 0)
    //{
    //    pca9685_interface_debug_print("pca9685: output frequency convert to register failed.\n");
    //    (void)pca9685_deinit(&gs_handle);
    //    
    //    return 1;
    //}
    
    /* set pre scale */
    //res = pca9685_set_prescaler(&gs_handle, reg);
    //if (res != 0)
    //{
    //    pca9685_interface_debug_print("pca9685: set pre scale failed.\n");
    //    (void)pca9685_deinit(&gs_handle);
    //    
    //    return 1;
    //}
    
    /* disable external clock pin */
    //res = pca9685_set_external_clock_pin(&gs_handle, PCA9685_BOOL_FALSE);
    //if (res != 0)
    //{
    //    pca9685_interface_debug_print("pca9685: set external clock pin failed.\n");
    //    (void)pca9685_deinit(&gs_handle);
    //    
    //    return 1;
    //}
    
    /* enable auto increment */
    /*res = pca9685_set_register_auto_increment(&gs_handle, PCA9685_BOOL_TRUE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set register auto increment failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* disable respond sub address 1 */
    /*res = pca9685_set_respond_subaddress_1(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set respond sub address 1 failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* disable respond sub address 2 */
    /*res = pca9685_set_respond_subaddress_2(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set respond sub address 2 failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* disable respond sub address 3 */
    /*res = pca9685_set_respond_subaddress_3(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set respond sub address 3 failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* disable respond all call */
    /*res = pca9685_set_respond_all_call(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set respond all call failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* disable output invert */
    /*res = pca9685_set_output_invert(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set output invert failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* stop output change */
    /*res = pca9685_set_output_change(&gs_handle, PCA9685_OUTPUT_CHANGE_STOP);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set output change failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* totem pole driver */
    /*res = pca9685_set_output_driver(&gs_handle, PCA9685_OUTPUT_DRIVER_TOTEM_POLE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set output driver failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* high impedance */
    /*res = pca9685_set_output_disable_type(&gs_handle, PCA9685_OUTPUT_DISABLE_TYPE_HIGH_IMPEDANCE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set output disable type failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* set sleep mode */
    /*res = pca9685_set_sleep_mode(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set sleep mode failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
    
    /* active */
    /*res = pca9685_set_active(&gs_handle, PCA9685_BOOL_TRUE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set active failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }*/
        #if(0)
    /* output */
    for (i = 1; i < times + 1; i++)
    {
        /* convert data */
        res = pca9685_pwm_convert_to_register(&gs_handle, 0.0f, 2.5f + (float)(i) / (float)(times) * 10.0f,
                                              (uint16_t *)&on_count, (uint16_t *)&off_count);
        if (res != 0)
        {
            pca9685_interface_debug_print("pca9685: convert to register failed.\n");
            (void)pca9685_deinit(&gs_handle);
            
            return 1;
        }
        
        /* write channel */
        res = pca9685_write_channel(&gs_handle, channel, on_count, off_count);
        if (res != 0)
        {
            pca9685_interface_debug_print("pca9685: write channel failed.\n");
            (void)pca9685_deinit(&gs_handle);
            
            return 1;
        }
        
        /* output data */
        pca9685_interface_debug_print("pca9685: set channel %d %0.2f degrees.\n", channel, (float)(i) / (float)(times) * 180.0f);
        
        /* delay 1000 ms */
        pca9685_interface_delay_ms(1000);
    }
    
    /* output */

    for (i = 1; i < times + 1; i++)
    {
        /* convert data */
        res = pca9685_pwm_convert_to_register(&gs_handle, 0.0f, 2.5f + (float)(i) / (float)(times) * 10.0f,
                                              (uint16_t *)&on_count, (uint16_t *)&off_count);
        if (res != 0)
        {
            pca9685_interface_debug_print("pca9685: convert to register failed.\n");
            (void)pca9685_deinit(&gs_handle);
            
            return 1;
        }
        
        /* write all channel */
        res = pca9685_write_all_channel(&gs_handle, on_count, off_count);
        if (res != 0)
        {
            pca9685_interface_debug_print("pca9685: write all channel failed.\n");
            (void)pca9685_deinit(&gs_handle);
            
            return 1;
        }
        
        /* output data */
        pca9685_interface_debug_print("pca9685: set all channel %0.2f degrees.\n", (float)(i) / (float)(times) * 180.0f);
        
        /* delay 1000 ms */
        pca9685_interface_delay_ms(1000);
    }
    
    /* inactive */
    res = pca9685_set_active(&gs_handle, PCA9685_BOOL_FALSE);
    if (res != 0)
    {
        pca9685_interface_debug_print("pca9685: set active failed.\n");
        (void)pca9685_deinit(&gs_handle);
        
        return 1;
    }
    
    /* finish write test */
    pca9685_interface_debug_print("pca9685: finish write test.\n");
    (void)pca9685_deinit(&gs_handle);
    #endif
    return 0;
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
  //BSP_RGB_Init(0, 0, 0);
  //BSP_RGB_D_Init(0, 1, 1);
  //BSP_RGB_Init(1000, 1000, 1000);
  //putsUart0("Initiating LEDs Done.......\r\n");
  //BSP_RGB_onboard_Init(1, 1, 1);
  //putsUart0("Initiating LEDs Done.......\r\n");
  //BSP_i2c1_init();
  pca9685_interface_iic_init();
  pca9685_servo_init(PCA9685_ADDRESS_A000001,  PCA9685_CHANNEL_5, 5);
  //pca9685_servo_init();
  putsUart0("Initiating I2C Done.......\r\n");
  BSP_LCD_Init();
  putsUart0("Initiating LCD Done.......\r\n");
  BSP_LCD_FillScreen(BSP_LCD_Color565(0x0F, 0x0F, 0x0F));
  putsUart0("\r\n");
  //EnableInterrupts();
  IntMasterEnable();
  //BSP_Delay1ms(100);
  BSP_LCD_DrawString(0, 0, "Red=    ", BSP_LCD_Color565(255, 0, 0));
  BSP_LCD_SetCursor(4, 0);
  BSP_LCD_OutUDec(500, BSP_LCD_Color565(255, 0, 0));
  BSP_LCD_DrawString(0, 1, "Green=    ", BSP_LCD_Color565(0, 255, 0));
  BSP_LCD_SetCursor(6, 1);
  BSP_LCD_OutUDec(500, BSP_LCD_Color565(0, 255, 0));
  BSP_LCD_DrawString(0, 2, "Blue=    ", BSP_LCD_Color565(0, 0, 255));
  BSP_LCD_SetCursor(5, 2);
  BSP_LCD_OutUDec(500, BSP_LCD_Color565(0, 0, 255));
      // print joystick status
  BSP_LCD_DrawString(0, 3, "JoyX=    ", BSP_LCD_Color565(255, 255, 255));
  BSP_LCD_DrawString(0, 4, "JoyY=    ", BSP_LCD_Color565(255, 255, 255));
  
  /* Initialize and start a watchdog timer: */
  //if ( 0 == watchdogInit( 0, 10000) )
  //{
  //    putsUart0("Initialization of watchdog reloading failed\r\n");
  //}
  //SysTick80_Wait10ms(1);  // approximately 10 ms
  SysTick50_Wait10ms(1);  // approximately 10 ms
  // Create the queues.
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
