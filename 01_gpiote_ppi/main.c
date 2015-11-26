/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* 
* @defgroup gpiote_example_main main.c
* @{
* @ingroup nrf_gpiote_example
* @brief GPIOTE Example Application main file.
*
* This file contains the source code for a sample application using GPIOTE. 
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"

#define GPIO_OUTPUT_PIN_NUMBER  BSP_LED_0     /**< Pin number for output. */
#define GPIO_INPUT_PIN_NUMBER   BSP_BUTTON_0  /**< Pin number for output. */

static void led_blinking_setup()
{
    uint32_t input_evt_addr;
    uint32_t gpiote_task_addr;
    nrf_ppi_channel_t ppi_channel;
    
    // Configure GPIOTE OUT task
    nrf_drv_gpiote_out_config_t output_config =
    {
        .init_state     = NRF_GPIOTE_INITIAL_VALUE_HIGH,
        .task_pin       = true,                                                                       \
        .action         = NRF_GPIOTE_POLARITY_TOGGLE
    };
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(GPIO_OUTPUT_PIN_NUMBER, &output_config));
    
    // Configure GPIOTE IN event
    nrf_drv_gpiote_in_config_t input_config = 
    {    
        .is_watcher     = false,
        .hi_accuracy    = true,
        .pull           = NRF_GPIO_PIN_PULLUP,
        .sense          = NRF_GPIOTE_POLARITY_TOGGLE       
    };
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(GPIO_INPUT_PIN_NUMBER, &input_config, NULL));
    
    // Get the instance allocated for both OUT task and IN event
    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);
    input_evt_addr = nrf_drv_gpiote_in_event_addr_get(GPIO_INPUT_PIN_NUMBER);

    // Get a PPI channel from the PPI pool
    APP_ERROR_CHECK(nrf_drv_ppi_channel_alloc(&ppi_channel));        
    // Tie the GPIOTE IN event and OUT task through allocated PPI channel
    APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(ppi_channel, input_evt_addr, gpiote_task_addr));
    // Enable the allocated PPI channel
    APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(ppi_channel));    
    
    // Enable OUT task and IN event
    nrf_drv_gpiote_out_task_enable(GPIO_OUTPUT_PIN_NUMBER);
    nrf_drv_gpiote_in_event_enable(GPIO_INPUT_PIN_NUMBER, false);    
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize PPI driver
    APP_ERROR_CHECK(nrf_drv_ppi_init());  
    
    // Initialize the GPIOTE driver
    APP_ERROR_CHECK(nrf_drv_gpiote_init());
    
    // Configure the PPI and GPIOTE to connect the button to the LED
    led_blinking_setup();  
    
    while (true)
    {
        // No need to do anything other than sleep, PPI/GPIOTE is self-driven
        __WFE();        
    }
}


/** @} */
