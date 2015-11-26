/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

/**
 * @file
 * @brief File with example code presenting usage of TWI driver.
 *
 * @sa twi_list_example
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include "config.h"
#include "app_uart.h"
#include "nrf_drv_twi_mod.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_util_platform.h"
#include "mma7660.h"
#include "nrf_delay.h"
#include "nrf_drv_twi_mod.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_rtc.h"
#include <string.h>

#define NUMBER_OF_XFERS 16

#define SENSOR_POLL_RATE SAMPLES_PER_SEC_16
#define SENSOR_POLL_FREQ_MS 62
#define CC_VALUE ((32768*SENSOR_POLL_FREQ_MS)/1000)

/**
 * @brief TWI master instance
 *
 * Instance of TWI master driver that would be used for communication with simulated
 * eeprom memory.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);
static uint8_t m_rxbuf[3*NUMBER_OF_XFERS] = {0};
static uint8_t m_txbuf[1] = {0};

nrf_drv_rtc_t rtc0 = NRF_DRV_RTC_INSTANCE(0);
nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1);

void twim_sync_xfer_setup(void);

static void rtc_event_handler(nrf_drv_rtc_int_type_t int_type)
{
    int i;
    for (i = 0; i < 3*NUMBER_OF_XFERS; i++)
    {
        if(m_rxbuf[i] > 31) m_rxbuf[i] = m_rxbuf[i] | 0xE0;
    }

    printf("-----------------\n\r");
    for (i = 0; i < NUMBER_OF_XFERS; i++)
    {
        printf("%4i %4i %4i \n\r", (int8_t)m_rxbuf[3*i], (int8_t)m_rxbuf[3*i+1], (int8_t)m_rxbuf[3*i+2]);
    }
    memset(m_rxbuf, 0, sizeof(m_rxbuf));   
    
    twim_sync_xfer_setup();
    nrf_drv_rtc_cc_set(&rtc1, 0, NUMBER_OF_XFERS*CC_VALUE+100, true);    
}

uint32_t rtc_init(mma7660_mode_t sensor_poll_mode)
{
    nrf_ppi_channel_t ppi_channel;
    
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    
    nrf_drv_rtc_init(&rtc0, NULL, rtc_event_handler);
    nrf_drv_rtc_cc_set(&rtc0, 0, CC_VALUE, false);
    nrf_drv_rtc_enable(&rtc0);
    
    nrf_drv_ppi_channel_alloc(&ppi_channel);
    nrf_drv_ppi_channel_assign(ppi_channel, (uint32_t)&NRF_RTC0->EVENTS_COMPARE[0], (uint32_t)&NRF_TWIM0->TASKS_STARTTX);
    nrf_drv_ppi_channel_enable(ppi_channel);
    
    nrf_drv_ppi_channel_alloc(&ppi_channel);
    nrf_drv_ppi_channel_assign(ppi_channel, (uint32_t)&NRF_RTC0->EVENTS_COMPARE[0], (uint32_t)&NRF_RTC0->TASKS_CLEAR);
    nrf_drv_ppi_channel_enable(ppi_channel);
    
    nrf_drv_rtc_init(&rtc1, NULL, rtc_event_handler);
    nrf_drv_rtc_cc_set(&rtc1, 0, NUMBER_OF_XFERS*CC_VALUE+100, true);
    nrf_drv_rtc_enable(&rtc1);
    
    nrf_drv_ppi_channel_alloc(&ppi_channel);
    nrf_drv_ppi_channel_assign(ppi_channel, (uint32_t)&NRF_RTC1->EVENTS_COMPARE[0], (uint32_t)&NRF_RTC1->TASKS_CLEAR);
    nrf_drv_ppi_channel_enable(ppi_channel);
    
    nrf_drv_ppi_channel_alloc(&ppi_channel);
    nrf_drv_ppi_channel_assign(ppi_channel, (uint32_t)&NRF_RTC1->EVENTS_COMPARE[0], (uint32_t)&NRF_RTC0->TASKS_CLEAR);
    nrf_drv_ppi_channel_enable(ppi_channel);
    
    
    return NRF_SUCCESS;
}

void twi_cb_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){}

/**
 * @brief Initialize the master TWI
 *
 * Function used to initialize master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure
 */
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    do
    {
        ret = nrf_drv_twi_init(&m_twi_master, &config, twi_cb_handler, NULL);
        if(NRF_SUCCESS != ret)
        {
            break;
        }
        nrf_drv_twi_enable(&m_twi_master);
    }while(0);
    return ret;
}

/**
 * @brief Handle UART errors
 *
 * Simple function for handling any error from UART module.
 * See UART example for more information.
 *
 * @param[in] p_event Event structure
 */
static void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

uint32_t uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud460800
    };

    APP_UART_FIFO_INIT(&comm_params,
                       4,
                       512,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    return err_code;
}

void twim_sync_xfer_setup(void)
{
    nrf_drv_twi_xfer_desc_t xfer_desc;
    uint32_t err_code;
    
    xfer_desc.address = MMA7660_DEFAULT_ADDRESS;
    xfer_desc.type = NRF_DRV_TWI_XFER_TXRX;
    xfer_desc.primary_length = 1;
    xfer_desc.secondary_length = 3;
    xfer_desc.p_primary_buf = m_txbuf;
    xfer_desc.p_secondary_buf = m_rxbuf;
    
    uint32_t flags = NRF_DRV_TWI_FLAGS_HOLD_XFER | NRF_DRV_TWI_FLAGS_REPEATED_XFER | 
                     NRF_DRV_TWI_FLAGS_NO_XFER_EVT_HANDLER | NRF_DRV_TWI_FLAGS_RX_POSTINC;

    do {
    err_code = nrf_drv_twi_xfer(&m_twi_master, &xfer_desc, flags);
    } while (err_code == NRF_ERROR_BUSY);    
}

/**
 *  The begin of the journey
 */
int main(void)
{    
    /* Initialization of UART */
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    APP_ERROR_CHECK(nrf_drv_ppi_init());
    APP_ERROR_CHECK(uart_init());   
    APP_ERROR_CHECK(twi_master_init());        
    APP_ERROR_CHECK(mma7660_init(&m_twi_master, SENSOR_POLL_RATE));        
    
    twim_sync_xfer_setup();
    
    APP_ERROR_CHECK(rtc_init(SENSOR_POLL_RATE));
    
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;    
    while(1)
    {
        __sev();
        __wfe();
        __wfe();
        nrf_gpio_pin_toggle(18);
    }       
}

/** @} */ /* End of group twi_master_with_twis_slave_example */

