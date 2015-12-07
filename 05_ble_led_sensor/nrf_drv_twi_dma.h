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
 * The TWI master driver provides APIs on a higher level.
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

/**
 * @brief TWI master driver event types.
 */
typedef enum
{
    NRF_DRV_TWI_EVENT_DONE,
    NRF_DRV_TWI_EVENT_ADDRESS_NACK,
    NRF_DRV_TWI_EVENT_DATA_NACK
} nrf_drv_twi_evt_type_t;

/**
 * @brief Structure for a TWI event.
 */
typedef struct
{
    nrf_drv_twi_evt_type_t type;   ///< Event type.
    uint8_t *              p_data; ///< Pointer to transferred data.
    uint8_t                length; ///< Number of bytes transferred.
} nrf_drv_twi_evt_t;

/**
 * @brief TWI event handler prototype.
 */
typedef void (* nrf_drv_twi_evt_handler_t)(nrf_drv_twi_evt_t * p_event,
                                           void *              p_context);

/**
 * @brief Function for initializing the TWI instance.
 *
 * @param[in] p_instance      TWI instance.
 * @param[in] p_config        Initial configuration. If NULL, the default configuration is used.
 * @param[in] event_handler   Event handler provided by the user. If NULL, blocking mode is enabled.
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
 * The transmission will be stopped when an error or time-out occurs.
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
 * @retval NRF_ERROR_INTERNAL If an @ref NRF_DRV_TWI_ERROR or a time-out has occurred (only in blocking mode).
 */
ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t const *       p_data,
                          uint8_t               length,
                          bool                  no_stop);

/**
 * @brief Function for reading data from a TWI slave.
 *
 * Transmission will be stopped when error or time-out occurs.
 *
 * @param[in] p_instance TWI instance.
 * @param     address    Address of a specific slave device (only 7 LSB).
 * @param[in] p_data     Pointer to a receive buffer.
 * @param     length     Number of bytes to be received.
 * @param     no_stop    If set, the stop condition is not generated on the bus
 *                       after the transfer is successfully completed (making it
 *                       possible to make a repeated start in the next transfer).
 *
 * @retval NRF_SUCCESS        If the procedure was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an @ref NRF_DRV_TWI_ERROR or a time-out has occurred (only in blocking mode).
 */
ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t *             p_data,
                          uint8_t               length,
                          bool                  no_stop);

/**
 * @brief Function for getting transferred data count.
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
__STATIC_INLINE uint32_t nrf_drv_twi_task_address_get(
                                               nrf_drv_twi_t const * p_instance,
                                               uint32_t              task);

/**
 * @brief Function for returning the address of a specific TWI event.
 *
 * @param[in]  p_instance  TWI instance.
 * @param[in]  event       Event.
 *
 * @return     Event address.
 */
__STATIC_INLINE uint32_t nrf_drv_twi_event_address_get(
                                               nrf_drv_twi_t const * p_instance,
                                               uint32_t              event);

#ifndef SUPPRESS_INLINE_IMPLEMENTATION
__STATIC_INLINE uint32_t nrf_drv_twi_task_address_get(
                                               nrf_drv_twi_t const * p_instance,
                                               uint32_t              task)
{
    if (p_instance->use_easy_dma)
    {
        return (uint32_t)nrf_twim_task_address_get(p_instance->p_reg,
            (nrf_twim_task_t)task);
    }
    else
    {
        return (uint32_t)nrf_twi_task_address_get(p_instance->p_reg,
            (nrf_twi_task_t)task);
    }
}

__STATIC_INLINE uint32_t nrf_drv_twi_event_address_get(
                                               nrf_drv_twi_t const * p_instance,
                                               uint32_t              event)
{
    if (p_instance->use_easy_dma)
    {
        return (uint32_t)nrf_twim_event_address_get(p_instance->p_reg,
            (nrf_twim_event_t)event);
    }
    else
    {
        return (uint32_t)nrf_twi_event_address_get(p_instance->p_reg,
            (nrf_twi_event_t)event);
    }
}
#endif // SUPPRESS_INLINE_IMPLEMENTATION
/**
 *@}
 **/

#endif // NRF_DRV_TWI_H__
