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

/**@file
 * @addtogroup nrf_twi_master TWI master HAL and driver
 * @ingroup    nrf_drivers
 * @brief      TWI master APIs.
 *
 * The TWI and TWIM HALs provide basic APIs for accessing the registers of the
 * TWI and TWIM peripherals respectively.
 *
 * The TWI master driver provides APIs on a higher level.:
 *
 */

#ifndef NRF_DRV_TWI_H__
#define NRF_DRV_TWI_H__

#include "nordic_common.h"
#include "nrf_drv_config.h"
#include "nrf_twi.h"
#ifdef NRF52
    #include "nrf_twim.h"
#endif
#include "sdk_errors.h"

#if defined(NRF52)
    #define NRF_DRV_TWI_PERIPHERAL(id)           \
        (CONCAT_3(TWI, id, _USE_EASY_DMA) == 1 ? \
            (void *)CONCAT_2(NRF_TWIM, id)       \
          : (void *)CONCAT_2(NRF_TWI, id))
#else
    #define NRF_DRV_TWI_PERIPHERAL(id)  (void *)CONCAT_2(NRF_TWI, id)
#endif

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
/**
 * @defgroup nrf_drv_twi TWI master driver
 * @{
 * @ingroup nrf_twi_master
 * @brief   Multi-instance TWI master driver.
 */

/**
 * @brief TWI master driver instance structure.
 */
typedef struct
{
    void *  p_reg;       ///< Pointer to the TWI/TWIM peripheral instance register set.
    uint8_t drv_inst_idx; ///< Driver instance index.
    bool    use_easy_dma; ///< True when peripheral with EasyDMA (i.e. TWIM) shall be used.
} nrf_drv_twi_t;

/**
 * @brief Macro for creating a TWI master driver instance.
 */
#define NRF_DRV_TWI_INSTANCE(id)                        \
{                                                       \
    .p_reg        = NRF_DRV_TWI_PERIPHERAL(id),         \
    .drv_inst_idx = CONCAT_3(TWI, id, _INSTANCE_INDEX), \
    .use_easy_dma = CONCAT_3(TWI, id, _USE_EASY_DMA)    \
}

/**
 * @brief Structure for TWI master driver instance configuration.
 */
typedef struct
{
    uint32_t            scl;                ///< SCL pin number.
    uint32_t            sda;                ///< SDA pin number.
    nrf_twi_frequency_t frequency;          ///< TWI frequency.
    uint8_t             interrupt_priority; ///< Interrupt priority.
} nrf_drv_twi_config_t;

/**
 * @brief TWI master driver instance default configuration.
 */
#define NRF_DRV_TWI_DEFAULT_CONFIG(id)                            \
{                                                                 \
    .frequency          = CONCAT_3(TWI, id, _CONFIG_FREQUENCY),   \
    .scl                = CONCAT_3(TWI, id, _CONFIG_SCL),         \
    .sda                = CONCAT_3(TWI, id, _CONFIG_SDA),         \
    .interrupt_priority = CONCAT_3(TWI, id, _CONFIG_IRQ_PRIORITY) \
}

#define NRF_DRV_TWI_FLAGS_TX_POSTINC          (1UL << 0) /**< TX buffer address incremented after transfer. */
#define NRF_DRV_TWI_FLAGS_RX_POSTINC          (1UL << 1) /**< RX buffer address incremented after transfer. */
#define NRF_DRV_TWI_FLAGS_NO_XFER_EVT_HANDLER (1UL << 2) /**< Interrupt after each transfer is suppressed and event handler is not called. */
#define NRF_DRV_TWI_FLAGS_HOLD_XFER           (1UL << 3) /**< Setup but not start the transfer. */
#define NRF_DRV_TWI_FLAGS_REPEATED_XFER       (1UL << 4) /**< Flag indicates that transfer will be executed multiple times. */
#define NRF_DRV_TWI_FLAGS_TX_NO_STOP          (1UL << 5) /**< Flag indicates that TX transfer will not end with stop condition. */

/**
 * @brief TWI master driver event types.
 */
typedef enum
{
    NRF_DRV_TWI_EVT_DONE,
    NRF_DRV_TWI_EVT_SYNC_DONE,
    NRF_DRV_TWI_EVT_ADDRESS_NACK,
    NRF_DRV_TWI_EVT_DATA_NACK
} nrf_drv_twi_evt_type_t;

/**
 * @brief TWI master driver transfer types.
 */
typedef enum
{
    NRF_DRV_TWI_XFER_TX,
    NRF_DRV_TWI_XFER_RX,
    NRF_DRV_TWI_XFER_TXRX,
    NRF_DRV_TWI_XFER_TXTX
} nrf_drv_twi_xfer_type_t;

/**
 * @brief Structure for a TWI transfer descriptor.
 */
typedef struct
{
    nrf_drv_twi_xfer_type_t type;
    uint8_t                 address;
    uint8_t                 primary_length; ///< Number of bytes transferred.
    uint8_t                 secondary_length; ///< Number of bytes transferred.
    uint8_t *               p_primary_buf; ///< Pointer to transferred data.
    uint8_t *               p_secondary_buf; ///< Pointer to transferred data.
} nrf_drv_twi_xfer_desc_t;

/**
 * @brief Structure for a TWI event.
 */
typedef struct
{
    nrf_drv_twi_evt_type_t  type;   ///< Event type.
    nrf_drv_twi_xfer_desc_t xfer_desc;

    nrf_drv_twi_xfer_type_t xfer_type; ///< Transfer type.
    nrf_drv_twi_xfer_desc_t data;   ///< Transfer details.
    nrf_drv_twi_xfer_desc_t secondary_data;   ///< Transfer details.
} nrf_drv_twi_evt_t;

#define NRF_DRV_TWI_XFER_DESC_TX(addr, p_data, length)                 \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_TX,                                   \
        .address = addr,                                               \
        .primary_length = length,                                      \
        .p_primary_buf  = p_data,                                      \
    }

#define NRF_DRV_TWI_XFER_DESC_RX(addr, p_data, length)                 \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_RX,                                   \
        .address = addr,                                               \
        .primary_length = length,                                      \
        .p_primary_buf  = p_data,                                      \
    }

#define NRF_DRV_TWI_XFER_DESC_TXRX(addr, p_tx, tx_len, p_rx, rx_len)   \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_TXRX,                                 \
        .address = addr,                                               \
        .primary_length   = tx_len,                                    \
        .secondary_length = rx_len,                                    \
        .p_primary_buf    = p_tx,                                      \
        .p_secondary_buf  = p_rx,                                      \
    }

#define NRF_DRV_TWI_XFER_DESC_TXTX(addr, p_tx, tx_len, p_tx2, tx_len2) \
    {                                                                  \
        .type = NRF_DRV_TWI_XFER_TXTX,                                 \
        .address = addr,                                               \
        .primary_length   = tx_len,                                    \
        .secondary_length = tx_len2,                                   \
        .p_primary_buf    = p_tx,                                      \
        .p_secondary_buf  = p_tx2,                                     \
    }

/**
 * @brief Tasks exposed by the driver.
 */
typedef enum
{
    NRF_DRV_TWI_TASK_STARTRX = NRF_TWI_TASK_STARTRX,//!< Start RX task - common for TWI and TWIM.
    NRF_DRV_TWI_TASK_STARTTX = NRF_TWI_TASK_STARTTX,//!< Start TX task - common for TWI and TWIM.
    NRF_DRV_TWI_TASK_STOP    = NRF_TWI_TASK_STOP,   //!< Stop task - common for TWI and TWIM.
} nrf_drv_twi_task_t;

/**
 * @brief Events exposed by the driver.
 */
typedef enum
{
    NRF_DRV_TWI_EVENT_STOPPED = NRF_TWI_EVENT_STOPPED,//!< Stopped event - common for TWI and TWIM.
#ifdef TWIM_IN_USE
    NRF_DRV_TWI_EVENT_LASTTX = NRF_TWIM_EVENT_LASTTX,//!< NRF_DRV_TWI_TASK_STARTRX
#endif
} nrf_drv_twi_event_t;

/**
 * @brief TWI event handler prototype.
 */
typedef void (* nrf_drv_twi_evt_handler_t)(nrf_drv_twi_evt_t const * p_event,
                                           void *                    p_context);

/**
 * @brief Function for initializing the TWI instance.
 *
 * @param[in] p_instance      TWI instance.
 * @param[in] p_config        Initial configuration. If NULL, the default configuration is used.
 * @param[in] event_handler   Event handler provided by the user. If NULL, blocking mode is enabled.
 * @param[in] p_context       Context passed in event handler.
 *
 * @retval  NRF_SUCCESS If initialization was successful.
 * @retval  NRF_ERROR_INVALID_STATE If the driver is in invalid state.
 */
ret_code_t nrf_drv_twi_init(nrf_drv_twi_t const *        p_instance,
                            nrf_drv_twi_config_t const * p_config,
                            nrf_drv_twi_evt_handler_t    event_handler,
                            void *                       p_context);

/**
 * @brief Function for uninitializing the TWI.
 *
 * @param[in] p_instance  TWI instance.
 */
void nrf_drv_twi_uninit(nrf_drv_twi_t const * p_instance);

/**
 * @brief Function for enabling the TWI instance.
 *
 * @param[in] p_instance  TWI instance.
 */
void nrf_drv_twi_enable(nrf_drv_twi_t const * p_instance);

/**
 * @brief Function for disabling the TWI instance.
 *
 * @param[in] p_instance  TWI instance.
 */
void nrf_drv_twi_disable(nrf_drv_twi_t const * p_instance);

/**
 * @brief Function for sending data to a TWI slave.
 *
 * The transmission will be stopped when an error occurs. It is possible to queue up to 2 transfers.
 * If next transfer cannot be queued driver will return error code @ref NRF_ERROR_BUSY. In some cases
 * transfers will be linked together by HW (RX after TX without STOP - Easy DMA mode) so it is more
 * optimal to use queuing capability.
 *
 * @param[in] p_instance TWI instance.
 * @param     address    Address of a specific slave device (only 7 LSB).
 * @param[in] p_data     Pointer to a transmit buffer.
 * @param     length     Number of bytes to send.
 * @param     no_stop    If set, the stop condition is not generated on the bus
 *                       after the transfer is successfully completed (making it
 *                       possible to make a repeated start in the next transfer).
 *
 * @retval NRF_SUCCESS        If the procedure was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If error was detected by hardware.
 */
ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t const *       p_data,
                          uint8_t               length,
                          bool                  no_stop);

/**
 * @brief Function for reading data from a TWI slave.
 *
 * Transmission will be stopped when error occurs.It is possible to queue up to 2 transfers.
 * If next transfer cannot be queued driver will return error code @ref NRF_ERROR_BUSY. In some cases
 * transfers will be linked together by HW (RX after TX without STOP - Easy DMA mode) so it is more
 * efficient to use queuing capability.
 *
 * @param[in] p_instance TWI instance.
 * @param     address    Address of a specific slave device (only 7 LSB).
 * @param[in] p_data     Pointer to a receive buffer.
 * @param     length     Number of bytes to be received.
 *
 * @retval NRF_SUCCESS             If the procedure was successful.
 * @retval NRF_ERROR_BUSY          If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL      If error was detected by hardware.
 */
ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t *             p_data,
                          uint8_t               length);

/**
 * @brief Function for preparing synchronous transfers.
 *
 * Synchronous transfers can be triggered using PPI by any event. A transfer can consist of write
 * operation followed by reading (without STOP state), only writing or only reading. Post-increment
 * option can be used to specify is pointers to buffers shall be incremented after completion.
 * Additional option flags can be used to specify details. STOPPED event can be used to count number
 * of transfers. Once expected number of transfers is performed user shall communicate that to the
 * driver using @ref nrf_drv_twi_sync_xfer_completed call. Event handler will be called from the
 * context of  @ref nrf_drv_twi_sync_xfer_completed call in that case.
 *
 * @param[in] p_instance        TWI instance.
 * @param[in] p_xfer_desc       Ponter to transfer descriptor.
 * @param[in] flags             Transfer options. postinc for rx and tx
 *
 * @retval NRF_SUCCESS             If the procedure was successful.
 * @retval NRF_ERROR_BUSY          If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_NOT_SUPPORTED If provided parameters are not supported.
 */
ret_code_t nrf_drv_twi_sync_xfer(nrf_drv_twi_t const * p_instance,
                                 nrf_drv_twi_xfer_desc_t * p_xfer_desc,
                                 uint32_t              flags
                                 );

/**
 * @brief Function for getting transferred data count.
 *
 * @note Function provides valid results only in legacy mode.
 *
 * @param[in] p_instance TWI instance.
 *
 * @return     Data count.
 */
uint32_t nrf_drv_twi_data_count_get(nrf_drv_twi_t const * const p_instance);

/**
 * @brief Function for returning the address of a specific TWI task.
 *
 * @param[in]  p_instance TWI instance.
 * @param      task       Task.
 *
 * @return     Task address.
 */
uint32_t nrf_drv_twi_task_address_get(nrf_drv_twi_t const * p_instance, nrf_drv_twi_task_t task);

/**
 * @brief Function for returning the address of a specific TWI event.
 *
 * @param[in]  p_instance  TWI instance.
 * @param[in]  event       Event.
 *
 * @return     Event address.
 */
uint32_t nrf_drv_twi_event_address_get(nrf_drv_twi_t const * p_instance,
                                       nrf_drv_twi_event_t   event);
/**
 *@}
 **/

#endif // NRF_DRV_TWI_H__
