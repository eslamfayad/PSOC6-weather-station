/*
 * ADCTask.c
 *
 *  Created on: Oct 23, 2023
 *      Author: EslamFayad
 */
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "stdlib.h"
//#include "ADCTask.h"
#include <stdio.h>
#include "GUI.h"
#include "mtb_ssd1306.h"
#include "mtb_ssd1306_i2c.h"
//#include "cy8ckit_032.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "ADCTask.h"

#define ADC_EXAMPLE_MODE SINGLE_CHANNEL

#if defined(CY_DEVICE_PSOC6A512K)  /* if the target is CY8CPROTO-062S3-4343W or CY8CPROTO-064B0S3*/
/* Channel 0 input pin */
#define VPLUS_CHANNEL_0  (P10_3)
#else
#define VPLUS_CHANNEL_0  (P10_0)
#endif

#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

#if defined(CY_DEVICE_PSOC6A256K)  /* if the target is CY8CKIT-062S4 */
/* Channel 1 VPLUS input pin */
#define VPLUS_CHANNEL_1  (P10_1)
/* Channel 1 VREF input pin */
#define VREF_CHANNEL_1   (P10_2)
#else
/* Channel 1 VPLUS input pin */
#define VPLUS_CHANNEL_1  (P10_4)
/* Channel 1 VREF input pin */
#define VREF_CHANNEL_1   (P10_5)
#endif

/* Number of scans every time ADC read is initiated */
#define NUM_SCAN                    (1)

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */

/* Conversion factor */
#define MICRO_TO_MILLI_CONV_RATIO        (1000u)

/* Acquistion time in nanosecond */
#define ACQUISITION_TIME_NS              (1000u)

/* ADC Scan delay in millisecond */
#define ADC_SCAN_DELAY_MS                (200u)


/*******************************************************************************
*       Enumerated Types
*******************************************************************************/
/* ADC Channel constants*/
enum ADC_CHANNELS
{
  CHANNEL_0 = 0,
  CHANNEL_1,
  NUM_CHANNELS
} adc_channel;


/* Macro for ADC Channel configuration*/
#define SINGLE_CHANNEL 1
#define MULTI_CHANNEL  2


#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

/* Multichannel initialization function */
void adc_multi_channel_init(void);

/* Function to read input voltage from multiple channels */
void adc_multi_channel_process(void);

/* ADC Event Handler */
static void adc_event_handler(void* arg, cyhal_adc_event_t event);

#else /* ADC_EXAMPLE_MODE == SINGLE_CHANNEL */

/* Single channel initialization function*/
void adc_single_channel_init(void);

/* Function to read input voltage from channel 0 */
void adc_single_channel_process(void);

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */
/*******************************************************************************
* Global Variables
*******************************************************************************/
/* ADC Object */
cyhal_adc_t adc_obj;
extern int32_t adc_result_0;

/* ADC Channel 0 Object */
cyhal_adc_channel_t adc_chan_0_obj;

const cyhal_adc_config_t adc_config = {
        .continuous_scanning=false, // Continuous Scanning is disabled
        .average_count=1,           // Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   // VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  // VNEG for Single ended channel set to VSSA
        .resolution = 12u,          // 12-bit resolution
        .ext_vref = NC,             // No connection
        .bypass_pin = NC };       // No connection




void adctasksingle(void *arg)
{
	cy_rslt_t result;


    /* Initialize Channel 0 */
    adc_single_channel_init();


    /* Update ADC configuration */
        result = cyhal_adc_configure(&adc_obj, &adc_config);
        if(result != CY_RSLT_SUCCESS)
        {
            printf("ADC configuration update failed. Error: %ld\n", (long unsigned int)result);
            CY_ASSERT(0);
        }
      //  GUI_Clear();
        //GUI_DrawRect(0, 0, LCD_GetXSize() - 1, LCD_GetYSize() - 1);
       // GUI_DispStringAt("Temperature", 30, 7); //GUI_DispString("Temperature:");
       // GUI_DispStringAt("Humidity", 39, 32);

    for (;;)
    {
    	/* Sample input voltage at channel 0 */
    	        adc_single_channel_process();
    	        vTaskDelay(1000);
    	        /* 200ms delay between scans */
    	         //cyhal_system_delay_ms(ADC_SCAN_DELAY_MS);

    	       //  GUI_Delay(100);

    }

}
void adc_single_channel_init(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize ADC. The ADC block which can connect to the channel 0 input pin is selected */
    result = cyhal_adc_init(&adc_obj, VPLUS_CHANNEL_0, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* ADC channel configuration */
    const cyhal_adc_channel_config_t channel_config = {
            .enable_averaging = false,  // Disable averaging for channel
            .min_acquisition_ns = ACQUISITION_TIME_NS, // Minimum acquisition time set to 1us
            .enabled = true };          // Sample this channel when ADC performs a scan

    /* Initialize a channel 0 and configure it to scan the channel 0 input pin in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, VPLUS_CHANNEL_0,
                                          CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    printf("ADC is configured in single channel configuration\r\n\n");
    printf("Provide input voltage at the channel 0 input pin. \r\n\n");
}

void adc_single_channel_process(void)
{

    int16_t hum =72;
    /* Read input voltage, convert it to millivolts and print input voltage */
    adc_result_0 = cyhal_adc_read_uv(&adc_chan_0_obj) / MICRO_TO_MILLI_CONV_RATIO;
    printf("Outdoor Temperature: %4ldC\r\n", (long int)adc_result_0/10);
    printf("Humidity:   %4ld%% \r\n", (int)hum);

}





