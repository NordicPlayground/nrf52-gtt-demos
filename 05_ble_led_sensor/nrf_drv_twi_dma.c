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

#include "nrf_drv_twi_dma.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "app_util_platform.h"
#include "nrf_delay.h"

#include <stdio.h>


#define TWI0_IRQ_HANDLER    SPI0_TWI0_IRQHandler
#define TWI1_IRQ_HANDLER    SPI1_TWI1_IRQHandler

// This set of macros makes it possible to exclude parts of code when one type
// of supported peripherals is not used.
#if ((TWI0_ENABLED == 1 && TWI0_USE_EASY_DMA == 1) || \
     (TWI1_ENABLED == 1 && TWI1_USE_EASY_DMA == 1))
    #define TWIM_IN_USE
#endif
#if ((TWI0_ENABLED == 1 && TWI0_USE_EASY_DMA != 1) || \
     (TWI1_ENABLED == 1 && TWI1_USE_EASY_DMA != 1))
    #define TWI_IN_USE
#endif
#if (defined(TWIM_IN_USE) && defined(TWI_IN_USE))
    // TWIM and TWI combined
    #define CODE_FOR_TWIM(code) if (p_instance->use_easy_dma) { code }
    #define CODE_FOR_TWI(code)  else { code }
#elif (defined(TWIM_IN_USE) && !defined(TWI_IN_USE))
    // TWIM only
    #define CODE_FOR_TWIM(code) { code }
    #define CODE_FOR_TWI(code)
#elif (!defined(TWIM_IN_USE) && defined(TWI_IN_USE))
    // TWI only
    #define CODE_FOR_TWIM(code)
    #define CODE_FOR_TWI(code)  { code }
#else
    #error "Wrong configuration."
#endif


// ZMIEN
#define DISABLE_ALL  0xFFFFFFFF

#define SCL_PIN_CONF        ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)    \
                            | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)   \
                            | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)    \
                            | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)   \
                            | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos))
#define SDA_PIN_CONF        SCL_PIN_CONF

#define SCL_PIN_CONF_CLR    ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)    \
                            | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)   \
                            | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)    \
                            | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)   \
                            | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos))
#define SDA_PIN_CONF_CLR    SCL_PIN_CONF_CLR


typedef struct
{
    uint8_t * p_data;
    uint8_t   address;
    bool      no_stop;
    uint8_t   length;
    bool      tx;
} twi_xfer_req_t;

// Control block - driver instance local data.
typedef struct
{
    nrf_drv_twi_evt_handler_t handler;
    void *                    p_context;
    volatile uint32_t         int_mask;
    nrf_drv_state_t           state;
    volatile bool             transfer_in_progress;
    volatile bool             error;

    // [no need for 'volatile' attribute for the following members, as they
    //  are not concurrently used in IRQ handlers and main line code]
    bool      no_stop;
    uint8_t   length;
    uint8_t * p_data;

    bool      tx;

    volatile twi_xfer_req_t xfer;
    volatile twi_xfer_req_t next_xfer;
    uint8_t        next_xfer_addr;

    uint8_t   bytes_transferred;
} twi_control_block_t;


static twi_control_block_t m_cb[TWI_COUNT];

static nrf_drv_twi_config_t const m_default_config[TWI_COUNT] = {
#if (TWI0_ENABLED == 1)
    NRF_DRV_TWI_DEFAULT_CONFIG(0),
#endif
#if (TWI1_ENABLED == 1)
    NRF_DRV_TWI_DEFAULT_CONFIG(1),
#endif
};


static void twi_clear_bus(nrf_drv_twi_t const * const p_instance,
                          nrf_drv_twi_config_t const * p_config)
{
    NRF_GPIO->PIN_CNF[p_config->scl] = SCL_PIN_CONF;
    NRF_GPIO->PIN_CNF[p_config->sda] = SDA_PIN_CONF;

    nrf_gpio_pin_set(p_config->scl);
    nrf_gpio_pin_set(p_config->sda);

    NRF_GPIO->PIN_CNF[p_config->scl] = SCL_PIN_CONF_CLR;
    NRF_GPIO->PIN_CNF[p_config->sda] = SDA_PIN_CONF_CLR;

    nrf_delay_us(4);

    for(int i = 0; i < 9; i++)
    {
        if (nrf_gpio_pin_read(p_config->sda))
        {
            if(i == 0)
            {
                return;
            }
            else
            {
                break;
            }
        }
        nrf_gpio_pin_clear(p_config->scl);
        nrf_delay_us(4);
        nrf_gpio_pin_set(p_config->scl);
        nrf_delay_us(4);
    }
    nrf_gpio_pin_clear(p_config->sda);
    nrf_delay_us(4);
    nrf_gpio_pin_set(p_config->sda);
}


ret_code_t nrf_drv_twi_init(nrf_drv_twi_t const *        p_instance,
                            nrf_drv_twi_config_t const * p_config,
                            nrf_drv_twi_evt_handler_t    event_handler,
                            void *                       p_context)
{
    twi_control_block_t * p_cb  = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->state != NRF_DRV_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_config == NULL)
    {
        p_config = &m_default_config[p_instance->drv_inst_idx];
    }

    p_cb->handler   = event_handler;
    p_cb->p_context = p_context;
    p_cb->int_mask  = 0;
    p_cb->transfer_in_progress = false;
    p_cb->xfer.no_stop         = false;
    p_cb->xfer.p_data          = NULL;
    p_cb->next_xfer.p_data     = NULL;

//    nrf_gpio_cfg(p_config->scl,
//        NRF_GPIO_PIN_DIR_INPUT,
//        NRF_GPIO_PIN_INPUT_CONNECT,
//        NRF_GPIO_PIN_PULLUP,
//        NRF_GPIO_PIN_S0D1,
//        NRF_GPIO_PIN_NOSENSE);
//    nrf_gpio_cfg(p_config->sda,
//        NRF_GPIO_PIN_DIR_INPUT,
//        NRF_GPIO_PIN_INPUT_CONNECT,
//        NRF_GPIO_PIN_PULLUP,
//        NRF_GPIO_PIN_S0D1,
//        NRF_GPIO_PIN_NOSENSE);

    twi_clear_bus(p_instance, p_config);

    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is
       disabled, these pins must be configured in the GPIO peripheral.
    */
    NRF_GPIO->PIN_CNF[p_config->scl] = SCL_PIN_CONF;
    NRF_GPIO->PIN_CNF[p_config->sda] = SDA_PIN_CONF;

    CODE_FOR_TWIM
    (
        NRF_TWIM_Type * p_twim = p_instance->p_reg;
        nrf_twim_pins_set(p_twim, p_config->scl, p_config->sda);
        nrf_twim_frequency_set(p_twim,
            (nrf_twim_frequency_t)p_config->frequency);
    )
    CODE_FOR_TWI
    (
        NRF_TWI_Type * p_twi = p_instance->p_reg;
        nrf_twi_pins_set(p_twi, p_config->scl, p_config->sda);
        nrf_twi_frequency_set(p_twi,
            (nrf_twi_frequency_t)p_config->frequency);
    )

    if (p_cb->handler)
    {
        nrf_drv_common_irq_enable(nrf_drv_get_IRQn(p_instance->p_reg),
            p_config->interrupt_priority);
    }

    p_cb->state = NRF_DRV_STATE_INITIALIZED;

    return NRF_SUCCESS;
}

void nrf_drv_twi_uninit(nrf_drv_twi_t const * p_instance)
{
    twi_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    ASSERT(p_cb->state != NRF_DRV_STATE_UNINITIALIZED);

    if (p_cb->handler)
    {
        nrf_drv_common_irq_disable(nrf_drv_get_IRQn(p_instance->p_reg));
    }
    nrf_drv_twi_disable(p_instance);

    p_cb->state = NRF_DRV_STATE_UNINITIALIZED;
}

void nrf_drv_twi_enable(nrf_drv_twi_t const * p_instance)
{
    twi_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    ASSERT(p_cb->state == NRF_DRV_STATE_INITIALIZED);

    CODE_FOR_TWIM
    (
        NRF_TWIM_Type * p_twim = p_instance->p_reg;

        nrf_twim_enable(p_twim);
    )
    CODE_FOR_TWI
    (
        NRF_TWI_Type * p_twi = p_instance->p_reg;

        nrf_twi_enable(p_twi);
    )

    p_cb->state = NRF_DRV_STATE_POWERED_ON;
}

void nrf_drv_twi_disable(nrf_drv_twi_t const * p_instance)
{
    twi_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    ASSERT(p_cb->state != NRF_DRV_STATE_UNINITIALIZED);

    CODE_FOR_TWIM
    (
        NRF_TWIM_Type * p_twim = p_instance->p_reg;
        p_cb->int_mask = 0;
        nrf_twim_int_disable(p_twim, DISABLE_ALL);
        nrf_twim_shorts_disable(p_twim, DISABLE_ALL);
        nrf_twim_disable(p_twim);
    )
    CODE_FOR_TWI
    (
        NRF_TWI_Type * p_twi = p_instance->p_reg;
        nrf_twi_int_disable(p_twi, DISABLE_ALL);
        nrf_twi_shorts_disable(p_twi, DISABLE_ALL);
        nrf_twi_disable(p_twi);
    )

    p_cb->state = NRF_DRV_STATE_INITIALIZED;
}


#ifdef TWI_IN_USE
static bool twi_send_byte(NRF_TWI_Type * p_twi, twi_control_block_t * p_cb)
{
    if (p_cb->bytes_transferred < p_cb->xfer.length)
    {
        nrf_twi_txd_set(p_twi, p_cb->xfer.p_data[p_cb->bytes_transferred]);
        ++(p_cb->bytes_transferred);
    }
    else
    {
        if (p_cb->xfer.no_stop)
        {
            nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_SUSPEND);
            return false;
        }
        else
        {
            nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_STOP);
        }
    }

    return true;
}
static bool twi_receive_byte(NRF_TWI_Type * p_twi, twi_control_block_t * p_cb)
{
    if (p_cb->bytes_transferred < p_cb->xfer.length)
    {
        p_cb->xfer.p_data[p_cb->bytes_transferred] = nrf_twi_rxd_get(p_twi);
        ++(p_cb->bytes_transferred);

        if (p_cb->bytes_transferred == p_cb->xfer.length-1)
        {
            nrf_twi_shorts_set(p_twi, NRF_TWI_SHORT_BB_STOP_MASK);
        }

        nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_RESUME);
    }

    return true;
}

static bool twi_transfer(NRF_TWI_Type * p_twi, twi_control_block_t * p_cb)
{
    if (p_cb->error)
    {
        nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_ERROR);
        nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_TXDSENT);
        nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_RXDREADY);
    }
    else if (nrf_twi_event_check(p_twi, NRF_TWI_EVENT_ERROR))
    {
        nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_ERROR);
        nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_STOP);
        p_cb->error = true;
    }
    else
    {
        if (nrf_twi_event_check(p_twi, NRF_TWI_EVENT_TXDSENT))
        {
            nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_TXDSENT);
            if (p_cb->xfer.tx && !twi_send_byte(p_twi, p_cb))
            {
                return false;
            }
        }
        else if (nrf_twi_event_check(p_twi, NRF_TWI_EVENT_RXDREADY))
        {
            nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_RXDREADY);
            if (!(p_cb->xfer.tx) && !twi_receive_byte(p_twi, p_cb))
            {
                return false;
            }
        }
    }

    if (nrf_twi_event_check(p_twi, NRF_TWI_EVENT_STOPPED))
    {
        nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_STOPPED);
        return false;
    }

    return true;
}
#endif // TWI_IN_USE

static ret_code_t start_transfer(twi_control_block_t * p_cb,
                                 void *                p_reg,
                                 uint8_t               address,
                                 uint8_t const *       p_data,
                                 uint8_t               length,
                                 bool                  no_stop,
                                 bool                  tx)
{
    ASSERT(p_cb->state == NRF_DRV_STATE_POWERED_ON);

    ASSERT(p_data != NULL);
    ASSERT(length > 0);

    // Currently we don't support repeated starts after a read transfer.
    if (no_stop && !tx)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }

    CODE_FOR_TWIM
    (
        if (!nrf_drv_is_in_RAM(p_data))
        {
            return NRF_ERROR_INVALID_ADDR;
        }
        if (length > 255)
        {
            return NRF_ERROR_INVALID_LENGTH;
        }

    )
    CODE_FOR_TWI
    (
    )

    bool is_busy = false;
    bool is_linked = false;
    volatile twi_xfer_req_t * p_xfer = &p_cb->xfer;

    /* Block TWI interrupts. Local critical section. */
    CODE_FOR_TWIM
    (
        NRF_TWIM_Type * p_twim = (NRF_TWIM_Type *)p_reg;
        nrf_twim_int_disable(p_twim, DISABLE_ALL);
    )
    CODE_FOR_TWI
    (
        NRF_TWI_Type * p_twi = (NRF_TWI_Type *)p_reg;
        nrf_twi_int_disable(p_twi, DISABLE_ALL);
    )

    if (p_cb->xfer.p_data)
    {
        if (p_cb->handler)
        {
            if (p_cb->next_xfer.p_data)
            {
                is_busy = true;
            }
            else
            {
                p_xfer = &p_cb->next_xfer;
                is_linked = true;
            }
        }
        else
        {
            is_busy = true;
        }
    }
    if (!is_busy)
    {
        p_xfer->p_data =  (uint8_t *)p_data;
    }
    else if (is_busy)
    {
        /* It driver is busy reenable interrupts and return. */
        CODE_FOR_TWIM
        (
            NRF_TWIM_Type * p_twim = (NRF_TWIM_Type *)p_reg;
            nrf_twim_int_enable(p_twim, p_cb->int_mask);
        )
        CODE_FOR_TWI
        (
            NRF_TWI_Type * p_twi = (NRF_TWI_Type *)p_reg;
            nrf_twi_int_enable(p_twi, p_cb->int_mask);
        )
        return NRF_ERROR_BUSY;
    }
    p_xfer->address = address;
    p_xfer->length  = length;
    p_xfer->no_stop = no_stop;
    p_xfer->tx      = tx;

    p_cb->error   = false;

    ret_code_t ret_code = NRF_SUCCESS;

    CODE_FOR_TWIM
    (
        NRF_TWIM_Type * p_twim = (NRF_TWIM_Type *)p_reg;

        if (!p_cb->handler)
        {
            nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
            nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
            nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);
        }

        nrf_twim_address_set(p_twim, address);
        if (tx)
        {
            if (!is_linked)
            {
                nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_TXSTARTED);
                nrf_twim_tx_buffer_set(p_twim, p_data, length);

                nrf_twim_shorts_set(p_twim, (no_stop) ? NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK :
                        NRF_TWIM_SHORT_LASTTX_STOP_MASK);
                nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STARTTX);
                nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);

            }
            else
            {
                if (p_cb->xfer.tx)
                {
                    while(!nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_TXSTARTED));
                }
                nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_TXSTARTED);
                nrf_twim_tx_buffer_set(p_twim, p_data, length);
            }

            p_cb->int_mask = (p_cb->xfer.no_stop) ? NRF_TWIM_INT_LASTTX_MASK | NRF_TWIM_INT_ERROR_MASK:
                                             NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;

        }
        else
        {
            if (!is_linked)
            {
                nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_RXSTARTED);
                nrf_twim_rx_buffer_set(p_twim, (uint8_t *)p_data, length);
                nrf_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTRX_STOP_MASK);

                p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
                nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STARTRX);
                nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
            }
            else
            {
                //if previous transfer was rx pend until it's started.
                if (!p_cb->xfer.tx)
                {
                    while(!nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_RXSTARTED));
                }
                nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_RXSTARTED);
                nrf_twim_rx_buffer_set(p_twim, (uint8_t *)p_data, length);

                if (p_cb->xfer.no_stop)
                {
                    p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK |
                               NRF_TWIM_INT_ERROR_MASK   |
                               NRF_TWIM_INT_LASTTX_MASK;
                    nrf_twim_shorts_enable(p_twim, NRF_TWIM_SHORT_LASTTX_STARTRX_MASK |
                                                   NRF_TWIM_SHORT_LASTRX_STOP_MASK);
                    if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_LASTTX))
                    {
                        nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
                        nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
                        nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STARTRX);
                    }
                    nrf_twim_shorts_disable(p_twim, NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK);
                }
                else
                {
                    p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK |
                               NRF_TWIM_INT_ERROR_MASK;
                }
            }
        }

        // In case TWI is suspended resume its operation.
       // nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);


        if (p_cb->handler)
        {
            nrf_twim_int_enable(p_twim, p_cb->int_mask);
        }
        else
        {
            nrf_twim_event_t event = (no_stop) ? NRF_TWIM_EVENT_LASTTX : NRF_TWIM_EVENT_STOPPED;

            while (!nrf_twim_event_check(p_twim, event))
            {
                if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_ERROR))
                {
                    nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);
                    nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
                    event = NRF_TWIM_EVENT_STOPPED;
                }
            }

            uint32_t errorsrc =  nrf_twim_errorsrc_get_and_clear(p_twim);

            p_cb->xfer.p_data = NULL;

            if (errorsrc)
            {
                return NRF_ERROR_INTERNAL;
            }
        }
    )
    CODE_FOR_TWI
    (
        NRF_TWI_Type * p_twi = (NRF_TWI_Type *)p_reg;
        if (!is_linked)
        {


            nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_STOPPED);
            nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_ERROR);
            nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_TXDSENT);
            nrf_twi_event_clear(p_twi, NRF_TWI_EVENT_RXDREADY);
            nrf_twi_shorts_set(p_twi, 0);

            p_cb->bytes_transferred = 0;

            nrf_twi_address_set(p_twi, address);

            // In case TWI is suspended resume its operation.
            nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_RESUME);

            if (tx)
            {
                nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_STARTTX);
                twi_send_byte(p_twi, p_cb);
            }
            else
            {
                nrf_twi_task_trigger(p_twi, NRF_TWI_TASK_STARTRX);
                if (p_cb->xfer.length == 1 && !p_cb->xfer.no_stop)
                {
                    nrf_twi_shorts_set(p_twi, NRF_TWI_SHORT_BB_STOP_MASK);
                }
                else
                {
                    nrf_twi_shorts_set(p_twi, NRF_TWI_SHORT_BB_SUSPEND_MASK);
                }
            }

            if (p_cb->handler)
            {
                p_cb->int_mask = NRF_TWI_INT_STOPPED_MASK   |
                                NRF_TWI_INT_ERROR_MASK     |
                                NRF_TWI_INT_TXDSENT_MASK   |
                                NRF_TWI_INT_RXDREADY_MASK;
                nrf_twi_int_enable(p_twi, p_cb->int_mask);
            }
            else
            {
                while (twi_transfer(p_twi, p_cb))
                {}

                if (p_cb->error)
                {
                    //uint32_t errorsrc = nrf_twi_errorsrc_get_and_clear(p_twi);
                    ret_code = NRF_ERROR_INTERNAL;
                }

                p_cb->xfer.p_data = NULL;
            }
        }
        else
        {
            p_cb->int_mask = NRF_TWI_INT_STOPPED_MASK   |
                            NRF_TWI_INT_ERROR_MASK     |
                            NRF_TWI_INT_TXDSENT_MASK   |
                            NRF_TWI_INT_RXDREADY_MASK;
            nrf_twi_int_enable(p_twi, p_cb->int_mask);
        }
    )

    return ret_code;
}

ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t const *       p_data,
                          uint8_t               length,
                          bool                  no_stop)
{
    return start_transfer(&m_cb[p_instance->drv_inst_idx], p_instance->p_reg, address, p_data, length, no_stop, true);
}

ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t *             p_data,
                          uint8_t               length,
                          bool                  no_stop)
{
    return start_transfer(&m_cb[p_instance->drv_inst_idx], p_instance->p_reg, address, p_data, length, no_stop, false);
}

#ifdef TWIM_IN_USE
static void irq_handler_twim(NRF_TWIM_Type * p_twim, twi_control_block_t * p_cb)
{
    ASSERT(p_cb->handler);

    if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_ERROR))
    {
        nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);
        if (!nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_STOPPED))
        {
            p_cb->int_mask &= ~(NRF_TWIM_INT_ERROR_MASK | NRF_TWIM_INT_LASTTX_MASK);
            nrf_twim_int_disable(p_twim, NRF_TWIM_INT_ERROR_MASK |
                                         NRF_TWIM_INT_LASTTX_MASK);
            nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
            nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
            return;
        }
    }

    nrf_drv_twi_evt_t event;
    event.p_data = p_cb->xfer.p_data;

    if (nrf_twim_event_check(p_twim, NRF_TWIM_EVENT_STOPPED))
    {
        nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
        nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
        nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTRX);
        nrf_twim_shorts_set(p_twim, 0);

        event.length = p_cb->xfer.tx ? nrf_twim_txd_amount_get(p_twim)
                                : nrf_twim_rxd_amount_get(p_twim);
        p_cb->xfer.p_data = NULL;
        if (p_cb->next_xfer.p_data)
        {
            uint8_t * p_data = p_cb->next_xfer.p_data;
            p_cb->next_xfer.p_data = NULL;

            (void)start_transfer(p_cb, p_twim, p_cb->next_xfer.address, p_data, p_cb->next_xfer.length,
                                 p_cb->next_xfer.no_stop, p_cb->next_xfer.tx);
        }

    }
    else
    {
        nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
        if (!p_cb->xfer.no_stop)
        {
            return;
        }
        event.length = p_cb->xfer.length;

        p_cb->xfer = p_cb->next_xfer;
        if (p_cb->xfer.p_data)
        {
            p_cb->next_xfer.p_data = NULL;
            if (p_cb->xfer.tx)
            {
                if (!p_cb->xfer.no_stop)
                {
                    nrf_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_STOP_MASK);
                    p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK;
                    nrf_twim_int_disable(p_twim, DISABLE_ALL);
                    nrf_twim_int_enable(p_twim, NRF_TWIM_INT_STOPPED_MASK);
                }
                nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
                nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STARTTX);
            }
            else
            {
                nrf_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTRX_STOP_MASK);
                p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK;
                nrf_twim_int_disable(p_twim, DISABLE_ALL);
                nrf_twim_int_enable(p_twim, NRF_TWIM_INT_STOPPED_MASK);
            }
        }
        else
        {
            p_cb->int_mask = 0;
            nrf_twim_int_disable(p_twim, DISABLE_ALL);
        }
    }

    uint32_t errorsrc = nrf_twim_errorsrc_get_and_clear(p_twim);
    if (errorsrc & NRF_TWIM_ERROR_ADDRESS_NACK)
    {
        event.type = NRF_DRV_TWI_EVENT_ADDRESS_NACK;
    }
    else if (errorsrc & NRF_TWIM_ERROR_DATA_NACK)
    {
        event.type = NRF_DRV_TWI_EVENT_DATA_NACK;
    }
    else
    {
        event.type = NRF_DRV_TWI_EVENT_DONE;
    }

    p_cb->handler(&event, p_cb->p_context);
}
#endif // TWIM_IN_USE

#ifdef TWI_IN_USE
static void irq_handler_twi(NRF_TWI_Type * p_twi, twi_control_block_t * p_cb)
{
    ASSERT(p_cb->handler);

    if (twi_transfer(p_twi, p_cb))
    {
        return;
    }

    nrf_drv_twi_evt_t event;
    event.p_data = p_cb->xfer.p_data;
    event.length = p_cb->bytes_transferred;

    if (p_cb->error)
    {
        uint32_t errorsrc = nrf_twi_errorsrc_get_and_clear(p_twi);
        if (errorsrc & NRF_TWI_ERROR_ADDRESS_NACK)
        {
            event.type = NRF_DRV_TWI_EVENT_ADDRESS_NACK;
        }
        else if (errorsrc & NRF_TWI_ERROR_DATA_NACK)
        {
            event.type = NRF_DRV_TWI_EVENT_DATA_NACK;
        }
    }
    else
    {
        event.type = NRF_DRV_TWI_EVENT_DONE;
    }

    /* if linked transfer present schedule it.
     * Would be good to wrap it around critical section since now it can be interrupted by API called
     * from higher level interrupt. */
    p_cb->xfer.p_data = NULL;
    if (p_cb->next_xfer.p_data)
    {
        uint8_t * p_data = p_cb->next_xfer.p_data;
        p_cb->next_xfer.p_data = NULL;
        (void)start_transfer(p_cb, p_twi, p_cb->next_xfer.address, p_data, p_cb->next_xfer.length,
                             p_cb->next_xfer.no_stop, p_cb->next_xfer.tx);
    }
    p_cb->handler(&event, p_cb->p_context);
}
#endif // TWI_IN_USE

#if (TWI0_ENABLED == 1)
void TWI0_IRQ_HANDLER(void)
{
    #if (TWI0_USE_EASY_DMA == 1) && defined(NRF52)
        irq_handler_twim(NRF_TWIM0,
    #else
        irq_handler_twi(NRF_TWI0,
    #endif
            &m_cb[TWI0_INSTANCE_INDEX]);
}
#endif // (TWI0_ENABLED == 1)

#if (TWI1_ENABLED == 1)
void TWI1_IRQ_HANDLER(void)
{
    #if (TWI1_USE_EASY_DMA == 1)
        irq_handler_twim(NRF_TWIM1,
    #else
        irq_handler_twi(NRF_TWI1,
    #endif
            &m_cb[TWI1_INSTANCE_INDEX]);
}
#endif // (SPI1_ENABLED == 1)
