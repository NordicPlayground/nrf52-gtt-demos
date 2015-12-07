/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 *
 * @defgroup ble_sdk_srv_lss LED Sensor Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    LED Sensor Service implementation.
 *
 * @details The LED Sensor Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the S110 SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate S110 SoftDevice events to the LED Sensor Service module
 *       by calling the ble_lss_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_LSS_H__
#define BLE_LSS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_LSS_SERVICE 0x0001                      /**< The UUID of the LED Sensor Service. */
#define BLE_LSS_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the LED Sensor service module. */

/* Forward declaration of the ble_lss_t type. */
typedef struct ble_lss_s ble_lss_t;

/**@brief LED Sensor Service event handler type. */
typedef void (*ble_lss_data_handler_t) (ble_lss_t * p_lss, uint8_t * p_data, uint16_t length);

/**@brief LED Sensor Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_lss_init
 *          function.
 */
typedef struct
{
    ble_lss_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_lss_init_t;

/**@brief LED Sensor Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_lss_s
{
    uint8_t                  uuid_type;               /**< UUID type for LED Sensor Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of LED Sensor Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the S110 SoftDevice). */
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_lss_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the LED Sensor Service.
 *
 * @param[out] p_lss      LED Sensor Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_lss_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_lss or p_lss_init is NULL.
 */
uint32_t ble_lss_init(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init);

/**@brief Function for handling the LED Sensor Service's BLE events.
 *
 * @details The LED Sensor Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the LED Sensor Service event handler of the
 * application if necessary.
 *
 * @param[in] p_lss       LED Sensor Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_lss_on_ble_evt(ble_lss_t * p_lss, ble_evt_t * p_ble_evt);

/**@brief Function for sending a string to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in] p_lss       Pointer to the LED Sensor Service structure.
 * @param[in] p_data      Data to be sent.
 * @param[in] length      Length of the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_lss_on_sensor_change(ble_lss_t * p_lss, uint8_t * p_data, uint16_t length);

#endif // BLE_LSS_H__

/** @} */
