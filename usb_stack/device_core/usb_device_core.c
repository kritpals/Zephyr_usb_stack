/*
 * USB Device Core Layer
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "../include/usb_stack.h"
#include "../include/usb_stack_types.h"

LOG_MODULE_REGISTER(usb_device_core, CONFIG_USB_STACK_LOG_LEVEL);

/**
 * @brief Initialize USB device core
 */
int usb_device_core_init(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Initializing USB device core");
    
    /* Initialize device core state */
    dev->state = USB_STACK_STATE_DETACHED;
    dev->configuration = 0;
    dev->address = 0;
    
    /* Initialize endpoint management */
    for (int i = 0; i < USB_STACK_MAX_ENDPOINTS * 2; i++) {
        dev->endpoints[i].enabled = false;
        dev->endpoints[i].type = USB_STACK_EP_TYPE_CONTROL;
        dev->endpoints[i].max_packet_size = 0;
        dev->endpoints[i].interval = 0;
        dev->endpoints[i].stalled = false;
        sys_dlist_init(&dev->endpoints[i].transfer_queue);
        k_mutex_init(&dev->endpoints[i].queue_mutex);
        dev->endpoints[i].transfer_count = 0;
        dev->endpoints[i].error_count = 0;
    }
    
    /* Setup control endpoint (EP0) */
    dev->endpoints[0].enabled = true;  /* EP0 OUT */
    dev->endpoints[0].type = USB_STACK_EP_TYPE_CONTROL;
    dev->endpoints[0].max_packet_size = 64; /* Default for USB 2.0 */
    dev->endpoints[0].interval = 0;
    dev->endpoints[0].address = 0x00;
    
    dev->endpoints[1].enabled = true;  /* EP0 IN */
    dev->endpoints[1].type = USB_STACK_EP_TYPE_CONTROL;
    dev->endpoints[1].max_packet_size = 64; /* Default for USB 2.0 */
    dev->endpoints[1].interval = 0;
    dev->endpoints[1].address = 0x80;
    
    LOG_DBG("USB device core initialized successfully");
    return 0;
}

/**
 * @brief Deinitialize USB device core
 */
int usb_device_core_deinit(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Deinitializing USB device core");
    
    /* Reset device state */
    dev->state = USB_STACK_STATE_DETACHED;
    dev->configuration = 0;
    dev->address = 0;
    
    /* Reset endpoints */
    for (int i = 0; i < USB_STACK_MAX_ENDPOINTS * 2; i++) {
        dev->endpoints[i].enabled = false;
        dev->endpoints[i].stalled = false;
        dev->endpoints[i].transfer_count = 0;
        dev->endpoints[i].error_count = 0;
    }
    
    LOG_DBG("USB device core deinitialized");
    return 0;
}

/**
 * @brief Handle USB device state changes
 */
int usb_device_core_set_state(struct usb_stack_device *dev, usb_stack_device_state_t new_state)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    usb_stack_device_state_t old_state = dev->state;
    
    if (old_state == new_state) {
        return 0; /* No change */
    }
    
    LOG_INF("USB device state change: %d -> %d", old_state, new_state);
    
    dev->state = new_state;
    
    /* Handle state-specific actions */
    switch (new_state) {
    case USB_STACK_STATE_DETACHED:
        /* Reset configuration */
        dev->configuration = 0;
        dev->address = 0;
        break;
        
    case USB_STACK_STATE_ATTACHED:
        /* Device attached to host */
        break;
        
    case USB_STACK_STATE_POWERED:
        /* Device powered by host */
        break;
        
    case USB_STACK_STATE_DEFAULT:
        /* Device in default state after reset */
        dev->configuration = 0;
        break;
        
    case USB_STACK_STATE_ADDRESS:
        /* Device address assigned */
        break;
        
    case USB_STACK_STATE_CONFIGURED:
        /* Device configured and ready */
        break;
        
    case USB_STACK_STATE_SUSPENDED:
        /* Device suspended */
        break;
        
    default:
        LOG_WRN("Unknown USB device state: %d", new_state);
        break;
    }
    
    return 0;
}

/**
 * @brief Get current USB device state
 */
usb_stack_device_state_t usb_device_core_get_state(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return USB_STACK_STATE_DETACHED;
    }
    
    return dev->state;
}

/**
 * @brief Configure USB endpoint
 */
int usb_device_core_configure_endpoint(struct usb_stack_device *dev, 
                                      uint8_t ep_addr,
                                      usb_stack_ep_type_t type,
                                      uint16_t max_packet_size,
                                      uint8_t interval)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    
    if (ep_index >= USB_STACK_MAX_ENDPOINTS * 2) {
        LOG_ERR("Invalid endpoint index: %d", ep_index);
        return -EINVAL;
    }
    
    LOG_DBG("Configuring endpoint 0x%02x: type=%d, mps=%d, interval=%d",
            ep_addr, type, max_packet_size, interval);
    
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    
    ep->enabled = true;
    ep->type = type;
    ep->max_packet_size = max_packet_size;
    ep->interval = interval;
    ep->address = ep_addr;
    
    return 0;
}

/**
 * @brief Deconfigure USB endpoint
 */
int usb_device_core_deconfigure_endpoint(struct usb_stack_device *dev, uint8_t ep_addr)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    uint8_t ep_num = USB_STACK_EP_NUM(ep_addr);
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    
    if (ep_index >= USB_STACK_MAX_ENDPOINTS * 2) {
        LOG_ERR("Invalid endpoint index: %d", ep_index);
        return -EINVAL;
    }
    
    if (ep_num == 0) {
        LOG_WRN("Cannot deconfigure control endpoint");
        return -EINVAL;
    }
    
    LOG_DBG("Deconfiguring endpoint 0x%02x", ep_addr);
    
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    memset(ep, 0, sizeof(*ep));
    
    return 0;
}

/**
 * @brief Get endpoint information
 */
const struct usb_stack_endpoint *usb_device_core_get_endpoint_info(struct usb_stack_device *dev, 
                                                                  uint8_t ep_addr)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return NULL;
    }
    
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    
    if (ep_index >= USB_STACK_MAX_ENDPOINTS * 2) {
        LOG_ERR("Invalid endpoint index: %d", ep_index);
        return NULL;
    }
    
    return &dev->endpoints[ep_index];
}

/**
 * @brief Set device configuration
 */
int usb_device_core_set_configuration(struct usb_stack_device *dev, uint8_t config)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Setting USB device configuration: %d", config);
    
    dev->configuration = config;
    
    if (config > 0) {
        usb_device_core_set_state(dev, USB_STACK_STATE_CONFIGURED);
    } else {
        usb_device_core_set_state(dev, USB_STACK_STATE_ADDRESS);
    }
    
    return 0;
}

/**
 * @brief Get device configuration
 */
uint8_t usb_device_core_get_configuration(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return 0;
    }
    
    return dev->configuration;
}

/**
 * @brief Set interface alternate setting
 */
int usb_device_core_set_interface(struct usb_stack_device *dev, 
                                 uint8_t interface, 
                                 uint8_t alternate_setting)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_DBG("Setting interface %d alternate setting: %d", interface, alternate_setting);
    
    /* Note: The USB stack structure doesn't have interface/alternate_setting fields
     * This would typically be handled by the application layer */
    
    return 0;
}

/**
 * @brief Get interface alternate setting
 */
uint8_t usb_device_core_get_interface(struct usb_stack_device *dev, uint8_t interface)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return 0;
    }
    
    /* Note: The USB stack structure doesn't have interface/alternate_setting fields
     * This would typically be handled by the application layer */
    
    return 0; /* Default alternate setting */
}

/**
 * @brief Handle USB reset
 */
int usb_device_core_handle_reset(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Handling USB reset");
    
    /* Reset to default state */
    usb_device_core_set_state(dev, USB_STACK_STATE_DEFAULT);
    
    /* Reset configuration */
    dev->configuration = 0;
    dev->address = 0;
    
    /* Reconfigure control endpoint */
    usb_device_core_configure_endpoint(dev, 0x00, USB_STACK_EP_TYPE_CONTROL, 64, 0);
    usb_device_core_configure_endpoint(dev, 0x80, USB_STACK_EP_TYPE_CONTROL, 64, 0);
    
    return 0;
}

/**
 * @brief Handle USB suspend
 */
int usb_device_core_handle_suspend(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Handling USB suspend");
    
    usb_device_core_set_state(dev, USB_STACK_STATE_SUSPENDED);
    
    return 0;
}

/**
 * @brief Handle USB resume
 */
int usb_device_core_handle_resume(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Handling USB resume");
    
    /* Restore previous state (before suspend) */
    if (dev->configuration > 0) {
        usb_device_core_set_state(dev, USB_STACK_STATE_CONFIGURED);
    } else {
        usb_device_core_set_state(dev, USB_STACK_STATE_ADDRESS);
    }
    
    return 0;
}
