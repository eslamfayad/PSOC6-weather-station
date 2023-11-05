/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "stdlib.h"
//#include "/home/eslamfayad/mtw/mtb_shared/retarget-io/release-v1.5.0/cy_retarget_io.h"
#include "cy_retarget_io.h"
#include "mtb_ssd1306.h"
#include "GUI.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oledTask.h"
#include "ADCTask.h"
//#include "terasic_includes.h"
//#include "rh_temp.h"
//#include "light_sensor.h"
/*******************************************************************************
* Macros
*******************************************************************************/


#define CYBSP_DEBUG_UART_RX (P5_0)

#define CYBSP_DEBUG_UART_TX (P5_1)

#define CYBSP_I2C_SCL (P6_0)
#define CYBSP_D15 CYBSP_I2C_SCL
#define CYBSP_I2C_SDA (P6_1)
#define CYBSP_D14 CYBSP_I2C_SDA
#define OLED_TASK_STACK_SIZE        (1024*5)
#define ADC_TASK_STACK_SIZE        (1024*5)
#define OLED_TASK_PRIORITY          (configMAX_PRIORITIES - 3)
#define ADC_TASK_PRIORITY          (configMAX_PRIORITIES - 3)
/*
 * Macro to choose between single channel and multiple channel configuration of
 * ADC. Single channel configuration uses channel 0 in single ended mode.
 * Multiple channel configuration uses two channels, channel 0 in single ended
 * mode and channel 1 in differential mode.
 *
 * The default configuration is set to use single channel.
 * To use multiple channel configuration set ADC_EXAMPLE_MODE macro to MULTI_CHANNEL.
 *
 */

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;
/* Default ADC configuration */
/* Variable to store ADC conversion result from channel 0 */
    int32_t adc_result_0 = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    //cy_rslt_t result;
    cyhal_i2c_t i2c_obj;
    /* This enables RTOS aware debugging in OpenOCD */
        uxTopUsedPriority = configMAX_PRIORITIES - 1 ;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Initialize retarget-io to use the debug UART port */
        result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                         CY_RETARGET_IO_BAUDRATE);

        printf("io ok \r\n");
            /* retarget-io init failed. Stop program execution */
            if (result != CY_RSLT_SUCCESS)
            {
                CY_ASSERT(0);
            }
    printf("bsb ok \r\n");
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();


        /* Initialize the device and board peripherals */
        //result = cybsp_init();

        //CY_ASSERT(result == CY_RSLT_SUCCESS);
        /* Print message */
                    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
                    printf("\x1b[2J\x1b[;H");
                    printf("-----------------------------------------------------------\r\n");
                    printf("Connect Things with code \r\n");
                    printf("Hackster.io  &  infineon \r\n");
                    printf("-----------------------------------------------------------\r\n\n");

       /* Initialize the User LED */
       result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

        /* GPIO init failed. Stop program execution */
         if (result != CY_RSLT_SUCCESS)
             {
               CY_ASSERT(0);
             }

        /* Initialize the I2C to use with the OLED display */
        result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

        CY_ASSERT(result == CY_RSLT_SUCCESS);
        printf("i2c ok \r\n");
        /* Initialize the OLED display */
        result = mtb_ssd1306_init_i2c(&i2c_obj);

        CY_ASSERT(result == CY_RSLT_SUCCESS);
        printf("TFT ok \r\n");
        __enable_irq();
        GUI_Init();
        GUI_SetFont(GUI_FONT_13B_1);
        GUI_SetColor(GUI_WHITE);
        GUI_SetBkColor(GUI_BLACK);
        GUI_Clear();
        GUI_DispStringAt("PSOC6 Weather",20,5);
        GUI_DispStringAt(" Station",40,20);
        cyhal_system_delay_ms(1000);
        /* Clear the display */
         GUI_Clear();
         GUI_DispStringAt("ESLAM FAYAD",30,25);
         cyhal_system_delay_ms(1000);
         //GUI_Delay(1000);
        /* Create the OLED task */

           xTaskCreate( adctasksingle, "ADCTask", ADC_TASK_STACK_SIZE,  NULL,
        		   ADC_TASK_PRIORITY,  NULL);
           printf(" 1 Task Created successfully \r\n");
           xTaskCreate( oledTask, "OLEDTask", OLED_TASK_STACK_SIZE,  NULL,
                                   OLED_TASK_PRIORITY,  NULL);
           printf(" 2 Tasks Created successfully \r\n");
           /* Start the FreeRTOS scheduler. */
           vTaskStartScheduler();

           /* Should never get here. */
           CY_ASSERT(0);

}


/* [] END OF FILE */
