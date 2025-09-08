/*
 * USB Stack Main API Header
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USB_STACK_H
#define USB_STACK_H

#include "usb_stack_config.h"
#include "usb_stack_types.h"
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB Stack Main API
 * @defgroup usb_stack USB Stack API
 * @{
 */

/**
 * @brief Initialize the USB stack
 * 
 * @param dev USB stack device instance
 * @param config Device configuration
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_init(struct usb_stack_device *dev, 
                   const struct usb_stack_device_config *config);

/**
 * @brief Deinitialize the USB stack
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_deinit(struct usb_stack_device *dev);

/**
 * @brief Enable the USB device
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_enable(struct usb_stack_device *dev);

/**
 * @brief Disable the USB device
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_disable(struct usb_stack_device *dev);

/**
 * @brief Connect the USB device to the bus
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_connect(struct usb_stack_device *dev);

/**
 * @brief Disconnect the USB device from the bus
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_disconnect(struct usb_stack_device *dev);

/**
 * @brief Configure an endpoint
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @param type Endpoint type
 * @param max_packet_size Maximum packet size
 * @param interval Polling interval (for interrupt/isochronous endpoints)
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_configure_endpoint(struct usb_stack_device *dev,
                                uint8_t ep_addr,
                                usb_stack_ep_type_t type,
                                uint16_t max_packet_size,
                                uint8_t interval);

/**
 * @brief Deconfigure an endpoint
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_deconfigure_endpoint(struct usb_stack_device *dev, uint8_t ep_addr);

/**
 * @brief Stall an endpoint
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_stall_endpoint(struct usb_stack_device *dev, uint8_t ep_addr);

/**
 * @brief Clear stall condition on an endpoint
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_unstall_endpoint(struct usb_stack_device *dev, uint8_t ep_addr);

/**
 * @brief Submit a transfer request
 * 
 * @param dev USB stack device instance
 * @param transfer Transfer request
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_submit_transfer(struct usb_stack_device *dev, 
                             struct usb_stack_transfer *transfer);

/**
 * @brief Cancel a transfer request
 * 
 * @param dev USB stack device instance
 * @param transfer Transfer request
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_cancel_transfer(struct usb_stack_device *dev, 
                             struct usb_stack_transfer *transfer);

/**
 * @brief Wait for transfer completion
 * 
 * @param transfer Transfer request
 * @param timeout Timeout value
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_wait_transfer(struct usb_stack_transfer *transfer, k_timeout_t timeout);

/**
 * @brief Get current device state
 * 
 * @param dev USB stack device instance
 * @return Current device state
 */
usb_stack_device_state_t usb_stack_get_state(struct usb_stack_device *dev);

/**
 * @brief Get current device speed
 * 
 * @param dev USB stack device instance
 * @return Current device speed
 */
usb_stack_speed_t usb_stack_get_speed(struct usb_stack_device *dev);

/**
 * @brief Get device address
 * 
 * @param dev USB stack device instance
 * @return Device address
 */
uint8_t usb_stack_get_address(struct usb_stack_device *dev);

/**
 * @brief Get configuration value
 * 
 * @param dev USB stack device instance
 * @return Configuration value
 */
uint8_t usb_stack_get_configuration(struct usb_stack_device *dev);

/**
 * @brief Check if endpoint is stalled
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @return true if stalled, false otherwise
 */
bool usb_stack_is_endpoint_stalled(struct usb_stack_device *dev, uint8_t ep_addr);

/**
 * @brief Get endpoint configuration
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @return Pointer to endpoint configuration, NULL if not found
 */
struct usb_stack_endpoint *usb_stack_get_endpoint(struct usb_stack_device *dev, 
                                                 uint8_t ep_addr);

/**
 * @brief Initialize a transfer request
 * 
 * @param transfer Transfer request to initialize
 * @param ep_addr Endpoint address
 * @param buffer Data buffer
 * @param length Transfer length
 * @param callback Completion callback (optional)
 * @param user_data User data for callback
 */
void usb_stack_init_transfer(struct usb_stack_transfer *transfer,
                            uint8_t ep_addr,
                            uint8_t *buffer,
                            uint32_t length,
                            usb_stack_transfer_callback_t callback,
                            void *user_data);

/**
 * @brief Synchronous data transfer
 * 
 * @param dev USB stack device instance
 * @param ep_addr Endpoint address
 * @param buffer Data buffer
 * @param length Transfer length
 * @param timeout Timeout value
 * @return Number of bytes transferred on success, negative error code otherwise
 */
int usb_stack_transfer_sync(struct usb_stack_device *dev,
                           uint8_t ep_addr,
                           uint8_t *buffer,
                           uint32_t length,
                           k_timeout_t timeout);

/**
 * @brief Send control response
 * 
 * @param dev USB stack device instance
 * @param buffer Response data (NULL for status stage)
 * @param length Response length
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_control_response(struct usb_stack_device *dev,
                              uint8_t *buffer,
                              uint32_t length);

/**
 * @brief Stall control endpoint (send STALL)
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_control_stall(struct usb_stack_device *dev);

/**
 * @brief Trigger remote wakeup
 * 
 * @param dev USB stack device instance
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_remote_wakeup(struct usb_stack_device *dev);

/**
 * @brief Get Type-C connection state
 * 
 * @param dev USB stack device instance
 * @return Type-C connection state
 */
usb_stack_typec_state_t usb_stack_get_typec_state(struct usb_stack_device *dev);

/**
 * @brief Get Type-C orientation
 * 
 * @param dev USB stack device instance
 * @return Type-C orientation
 */
usb_stack_typec_orientation_t usb_stack_get_typec_orientation(struct usb_stack_device *dev);

/**
 * @brief Get PHY type
 * 
 * @param dev USB stack device instance
 * @return PHY type
 */
usb_stack_phy_type_t usb_stack_get_phy_type(struct usb_stack_device *dev);

/**
 * @brief Register event callback
 * 
 * @param dev USB stack device instance
 * @param callback Event callback function
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_register_event_callback(struct usb_stack_device *dev,
                                     usb_stack_event_callback_t callback);

/**
 * @brief Register setup callback
 * 
 * @param dev USB stack device instance
 * @param callback Setup callback function
 * @return USB_STACK_SUCCESS on success, error code otherwise
 */
int usb_stack_register_setup_callback(struct usb_stack_device *dev,
                                     usb_stack_setup_callback_t callback);

/**
 * @brief Get device statistics
 * 
 * @param dev USB stack device instance
 * @param reset_count Pointer to store reset count (optional)
 * @param suspend_count Pointer to store suspend count (optional)
 * @param resume_count Pointer to store resume count (optional)
 * @param setup_count Pointer to store setup count (optional)
 * @param transfer_count Pointer to store transfer count (optional)
 * @param error_count Pointer to store error count (optional)
 */
void usb_stack_get_statistics(struct usb_stack_device *dev,
                             uint32_t *reset_count,
                             uint32_t *suspend_count,
                             uint32_t *resume_count,
                             uint32_t *setup_count,
                             uint32_t *transfer_count,
                             uint32_t *error_count);

/**
 * @brief Reset device statistics
 * 
 * @param dev USB stack device instance
 */
void usb_stack_reset_statistics(struct usb_stack_device *dev);

/**
 * @}
 */

/* Logging macros */
#define USB_STACK_LOG_MODULE_NAME usb_stack
LOG_MODULE_DECLARE(USB_STACK_LOG_MODULE_NAME, CONFIG_USB_STACK_LOG_LEVEL);

#define USB_STACK_LOG_ERR(...) LOG_ERR(__VA_ARGS__)
#define USB_STACK_LOG_WRN(...) LOG_WRN(__VA_ARGS__)
#define USB_STACK_LOG_INF(...) LOG_INF(__VA_ARGS__)
#define USB_STACK_LOG_DBG(...) LOG_DBG(__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* USB_STACK_H */
