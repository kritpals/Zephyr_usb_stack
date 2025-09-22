/*
 * USB Adaptation Layer
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_ch9.h>
#include <string.h>

#include "../include/usb_stack.h"
#include "../include/usb_stack_types.h"

/* USB Request Type Recipient Masks - Define if not available */
#ifndef USB_RECIP_MASK
#define USB_RECIP_MASK          0x1F
#endif

#ifndef USB_RECIP_DEVICE
#define USB_RECIP_DEVICE        0x00
#endif

#ifndef USB_RECIP_INTERFACE
#define USB_RECIP_INTERFACE     0x01
#endif

#ifndef USB_RECIP_ENDPOINT
#define USB_RECIP_ENDPOINT      0x02
#endif

/* USB Standard Requests - Define if not available */
#ifndef USB_REQ_GET_STATUS
#define USB_REQ_GET_STATUS      0x00
#endif

#ifndef USB_REQ_CLEAR_FEATURE
#define USB_REQ_CLEAR_FEATURE   0x01
#endif

#ifndef USB_REQ_SET_FEATURE
#define USB_REQ_SET_FEATURE     0x03
#endif

#ifndef USB_REQ_SET_ADDRESS
#define USB_REQ_SET_ADDRESS     0x05
#endif

#ifndef USB_REQ_GET_DESCRIPTOR
#define USB_REQ_GET_DESCRIPTOR  0x06
#endif

#ifndef USB_REQ_SET_DESCRIPTOR
#define USB_REQ_SET_DESCRIPTOR  0x07
#endif

#ifndef USB_REQ_GET_CONFIGURATION
#define USB_REQ_GET_CONFIGURATION 0x08
#endif

#ifndef USB_REQ_SET_CONFIGURATION
#define USB_REQ_SET_CONFIGURATION 0x09
#endif

#ifndef USB_REQ_GET_INTERFACE
#define USB_REQ_GET_INTERFACE   0x0A
#endif

#ifndef USB_REQ_SET_INTERFACE
#define USB_REQ_SET_INTERFACE   0x0B
#endif

#ifndef USB_REQ_SYNCH_FRAME
#define USB_REQ_SYNCH_FRAME     0x0C
#endif

/* USB Feature Selectors - Define if not available */
#ifndef USB_FEATURE_ENDPOINT_HALT
#define USB_FEATURE_ENDPOINT_HALT 0x00
#endif

#ifndef USB_FEATURE_DEVICE_REMOTE_WAKEUP
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP 0x01
#endif

LOG_MODULE_REGISTER(usb_adaptation, CONFIG_USB_STACK_LOG_LEVEL);

/**
 * @brief Initialize USB adaptation layer
 */
int usb_adaptation_init(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Initializing USB adaptation layer");
    
    /* Initialize device configuration if not already done */
    if (!dev->config) {
        LOG_WRN("Device configuration not set");
        return -EINVAL;
    }
    
    /* Initialize device state */
    dev->state = USB_STACK_STATE_DETACHED;
    dev->configuration = 0;
    dev->address = 0;
    
    /* Initialize control buffer if needed */
    if (!dev->control_buffer) {
        dev->control_buffer = k_malloc(USB_STACK_CTRL_BUF_SIZE);
        if (!dev->control_buffer) {
            LOG_ERR("Failed to allocate control buffer");
            return -ENOMEM;
        }
    }
    dev->control_length = 0;
    
    LOG_DBG("USB adaptation layer initialized successfully");
    return 0;
}

/**
 * @brief Deinitialize USB adaptation layer
 */
int usb_adaptation_deinit(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Deinitializing USB adaptation layer");
    
    /* Free control buffer if allocated */
    if (dev->control_buffer) {
        k_free(dev->control_buffer);
        dev->control_buffer = NULL;
    }
    dev->control_length = 0;
    
    /* Reset device state */
    dev->state = USB_STACK_STATE_DETACHED;
    dev->configuration = 0;
    dev->address = 0;
    
    LOG_DBG("USB adaptation layer deinitialized");
    return 0;
}

/**
 * @brief Register a USB class handler (simplified version)
 */
int usb_adaptation_register_class_handler(struct usb_stack_device *dev,
                                         uint8_t class_code,
                                         usb_stack_setup_callback_t handler,
                                         void *user_data)
{
    if (!dev || !handler) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    /* For simplified implementation, just log the registration
     * In a real implementation, the setup callback would be set during device initialization */
    LOG_INF("Registered class handler for class 0x%02x", class_code);
    return 0;
}

/**
 * @brief Unregister a USB class handler (simplified version)
 */
int usb_adaptation_unregister_class_handler(struct usb_stack_device *dev, uint8_t class_code)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    /* For simplified implementation, just log the unregistration */
    LOG_INF("Unregistered class handler for class 0x%02x", class_code);
    return 0;
}

/**
 * @brief Add a USB interface (simplified version)
 */
int usb_adaptation_add_interface(struct usb_stack_device *dev,
                                uint8_t interface_number,
                                uint8_t alternate_setting,
                                uint8_t class_code,
                                uint8_t subclass_code,
                                uint8_t protocol_code,
                                uint8_t interface_string)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    /* For simplified implementation, just log the interface addition */
    LOG_INF("Added interface %d: class=0x%02x, subclass=0x%02x, protocol=0x%02x",
            interface_number, class_code, subclass_code, protocol_code);
    
    return 0;
}

/**
 * @brief Remove a USB interface (simplified version)
 */
int usb_adaptation_remove_interface(struct usb_stack_device *dev, uint8_t interface_number)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    /* For simplified implementation, just log the interface removal */
    LOG_INF("Removed interface %d", interface_number);
    return 0;
}

/**
 * @brief Get interface descriptor (simplified version)
 */
const struct usb_interface_descriptor *usb_adaptation_get_interface(struct usb_stack_device *dev,
                                                                   uint8_t interface_number)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return NULL;
    }
    
    /* For simplified implementation, return NULL - interface management
     * would be handled by the application layer in a real implementation */
    return NULL;
}

/**
 * @brief Handle setup packet
 */
int usb_adaptation_handle_setup_packet(struct usb_stack_device *dev,
                                      const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    LOG_DBG("Handling setup packet: bmRequestType=0x%02x, bRequest=0x%02x, wValue=0x%04x, wIndex=0x%04x, wLength=%d",
            setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);
    
    /* Check if this is an interface-specific request */
    if ((setup->bmRequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
        uint8_t interface_number = setup->wIndex & 0xFF;
        
        /* For simplified implementation, forward to the device's setup callback if available */
        if (dev->config && dev->config->setup_callback) {
            return dev->config->setup_callback(dev, (struct usb_setup_packet *)setup);
        }
        
        LOG_WRN("No class handler for interface %d", interface_number);
        return -ENOTSUP;
    }
    
    /* Handle standard device requests */
    switch (setup->bRequest) {
    case USB_REQ_GET_STATUS:
        return usb_adaptation_handle_get_status(dev, setup);
        
    case USB_REQ_CLEAR_FEATURE:
        return usb_adaptation_handle_clear_feature(dev, setup);
        
    case USB_REQ_SET_FEATURE:
        return usb_adaptation_handle_set_feature(dev, setup);
        
    case USB_REQ_SET_ADDRESS:
        return usb_adaptation_handle_set_address(dev, setup);
        
    case USB_REQ_GET_DESCRIPTOR:
        return usb_adaptation_handle_get_descriptor(dev, setup);
        
    case USB_REQ_SET_DESCRIPTOR:
        return usb_adaptation_handle_set_descriptor(dev, setup);
        
    case USB_REQ_GET_CONFIGURATION:
        return usb_adaptation_handle_get_configuration(dev, setup);
        
    case USB_REQ_SET_CONFIGURATION:
        return usb_adaptation_handle_set_configuration(dev, setup);
        
    case USB_REQ_GET_INTERFACE:
        return usb_adaptation_handle_get_interface(dev, setup);
        
    case USB_REQ_SET_INTERFACE:
        return usb_adaptation_handle_set_interface(dev, setup);
        
    case USB_REQ_SYNCH_FRAME:
        return usb_adaptation_handle_synch_frame(dev, setup);
        
    default:
        LOG_WRN("Unsupported setup request: 0x%02x", setup->bRequest);
        return -ENOTSUP;
    }
}

/**
 * @brief Handle GET_STATUS request
 */
int usb_adaptation_handle_get_status(struct usb_stack_device *dev,
                                    const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    static uint16_t status_data = 0;
    
    switch (setup->bmRequestType & USB_RECIP_MASK) {
    case USB_RECIP_DEVICE:
        /* Device status */
        status_data = 0;
        if (dev->config && dev->config->self_powered) {
            status_data |= 0x01; /* Self-powered bit */
        }
        break;
        
    case USB_RECIP_INTERFACE:
        /* Interface status (always 0) */
        status_data = 0;
        break;
        
    case USB_RECIP_ENDPOINT:
        /* Endpoint status */
        {
            uint8_t ep_addr = setup->wIndex & 0xFF;
            status_data = usb_function_driver_is_endpoint_stalled(dev, ep_addr) ? 1 : 0;
        }
        break;
        
    default:
        LOG_ERR("Invalid recipient for GET_STATUS");
        return -EINVAL;
    }
    
    /* Send status data */
    return usb_function_driver_submit_transfer(dev, 0x80, (uint8_t *)&status_data, 2, NULL, NULL);
}

/**
 * @brief Handle CLEAR_FEATURE request
 */
int usb_adaptation_handle_clear_feature(struct usb_stack_device *dev,
                                       const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    switch (setup->bmRequestType & USB_RECIP_MASK) {
    case USB_RECIP_DEVICE:
        /* Device features */
        switch (setup->wValue) {
        case USB_FEATURE_DEVICE_REMOTE_WAKEUP:
            /* Clear remote wakeup */
            LOG_INF("Clearing device remote wakeup");
            break;
            
        default:
            LOG_WRN("Unsupported device feature: %d", setup->wValue);
            return -ENOTSUP;
        }
        break;
        
    case USB_RECIP_ENDPOINT:
        /* Endpoint features */
        switch (setup->wValue) {
        case USB_FEATURE_ENDPOINT_HALT:
            {
                uint8_t ep_addr = setup->wIndex & 0xFF;
                LOG_INF("Clearing endpoint %d halt", ep_addr & 0x0F);
                return usb_function_driver_clear_stall(dev, ep_addr);
            }
            
        default:
            LOG_WRN("Unsupported endpoint feature: %d", setup->wValue);
            return -ENOTSUP;
        }
        break;
        
    default:
        LOG_ERR("Invalid recipient for CLEAR_FEATURE");
        return -EINVAL;
    }
    
    /* Send status stage */
    return usb_function_driver_submit_transfer(dev, 0x80, NULL, 0, NULL, NULL);
}

/**
 * @brief Handle SET_FEATURE request
 */
int usb_adaptation_handle_set_feature(struct usb_stack_device *dev,
                                     const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    switch (setup->bmRequestType & USB_RECIP_MASK) {
    case USB_RECIP_DEVICE:
        /* Device features */
        switch (setup->wValue) {
        case USB_FEATURE_DEVICE_REMOTE_WAKEUP:
            /* Set remote wakeup */
            LOG_INF("Setting device remote wakeup");
            break;
            
        default:
            LOG_WRN("Unsupported device feature: %d", setup->wValue);
            return -ENOTSUP;
        }
        break;
        
    case USB_RECIP_ENDPOINT:
        /* Endpoint features */
        switch (setup->wValue) {
        case USB_FEATURE_ENDPOINT_HALT:
            {
                uint8_t ep_addr = setup->wIndex & 0xFF;
                LOG_INF("Setting endpoint %d halt", ep_addr & 0x0F);
                return usb_function_driver_stall_endpoint(dev, ep_addr);
            }
            
        default:
            LOG_WRN("Unsupported endpoint feature: %d", setup->wValue);
            return -ENOTSUP;
        }
        break;
        
    default:
        LOG_ERR("Invalid recipient for SET_FEATURE");
        return -EINVAL;
    }
    
    /* Send status stage */
    return usb_function_driver_submit_transfer(dev, 0x80, NULL, 0, NULL, NULL);
}

/**
 * @brief Handle SET_ADDRESS request
 */
int usb_adaptation_handle_set_address(struct usb_stack_device *dev,
                                     const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t address = setup->wValue & 0x7F;
    
    LOG_INF("Setting device address: %d", address);
    
    /* Set address using USB stack */
    dev->address = address;
    
    /* Update device state */
    if (address == 0) {
        usb_device_core_set_state(dev, USB_STACK_STATE_DEFAULT);
    } else {
        usb_device_core_set_state(dev, USB_STACK_STATE_ADDRESS);
    }
    
    /* Send status stage */
    return usb_function_driver_submit_transfer(dev, 0x80, NULL, 0, NULL, NULL);
}

/**
 * @brief Handle GET_DESCRIPTOR request
 */
int usb_adaptation_handle_get_descriptor(struct usb_stack_device *dev,
                                        const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t descriptor_type = (setup->wValue >> 8) & 0xFF;
    uint8_t descriptor_index = setup->wValue & 0xFF;
    
    LOG_DBG("GET_DESCRIPTOR: type=0x%02x, index=%d, length=%d",
            descriptor_type, descriptor_index, setup->wLength);
    
    /* This is a simplified implementation - in a real system,
     * you would have proper descriptor management */
    
    switch (descriptor_type) {
    case USB_DESC_DEVICE:
        /* Return device descriptor */
        LOG_DBG("Returning device descriptor");
        /* TODO: Implement proper descriptor handling */
        break;
        
    case USB_DESC_CONFIGURATION:
        /* Return configuration descriptor */
        LOG_DBG("Returning configuration descriptor");
        /* TODO: Implement proper descriptor handling */
        break;
        
    case USB_DESC_STRING:
        /* Return string descriptor */
        LOG_DBG("Returning string descriptor %d", descriptor_index);
        /* TODO: Implement proper descriptor handling */
        break;
        
    default:
        LOG_WRN("Unsupported descriptor type: 0x%02x", descriptor_type);
        return -ENOTSUP;
    }
    
    /* For now, just send empty response */
    return usb_function_driver_submit_transfer(dev, 0x80, NULL, 0, NULL, NULL);
}

/**
 * @brief Handle SET_DESCRIPTOR request
 */
int usb_adaptation_handle_set_descriptor(struct usb_stack_device *dev,
                                        const struct usb_setup_packet *setup)
{
    /* SET_DESCRIPTOR is optional and rarely used */
    LOG_WRN("SET_DESCRIPTOR not supported");
    return -ENOTSUP;
}

/**
 * @brief Handle GET_CONFIGURATION request
 */
int usb_adaptation_handle_get_configuration(struct usb_stack_device *dev,
                                           const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t config = usb_device_core_get_configuration(dev);
    
    LOG_DBG("GET_CONFIGURATION: returning %d", config);
    
    return usb_function_driver_submit_transfer(dev, 0x80, &config, 1, NULL, NULL);
}

/**
 * @brief Handle SET_CONFIGURATION request
 */
int usb_adaptation_handle_set_configuration(struct usb_stack_device *dev,
                                           const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t config = setup->wValue & 0xFF;
    
    LOG_INF("SET_CONFIGURATION: %d", config);
    
    /* Set configuration in device core */
    int ret = usb_device_core_set_configuration(dev, config);
    if (ret) {
        LOG_ERR("Failed to set configuration: %d", ret);
        return ret;
    }
    
    /* Configuration is already stored in dev->configuration by device core */
    
    /* Send status stage */
    return usb_function_driver_submit_transfer(dev, 0x80, NULL, 0, NULL, NULL);
}

/**
 * @brief Handle GET_INTERFACE request
 */
int usb_adaptation_handle_get_interface(struct usb_stack_device *dev,
                                       const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t interface_number = setup->wIndex & 0xFF;
    uint8_t alternate_setting = usb_device_core_get_interface(dev, interface_number);
    
    LOG_DBG("GET_INTERFACE: interface=%d, alternate=%d", interface_number, alternate_setting);
    
    return usb_function_driver_submit_transfer(dev, 0x80, &alternate_setting, 1, NULL, NULL);
}

/**
 * @brief Handle SET_INTERFACE request
 */
int usb_adaptation_handle_set_interface(struct usb_stack_device *dev,
                                       const struct usb_setup_packet *setup)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t interface_number = setup->wIndex & 0xFF;
    uint8_t alternate_setting = setup->wValue & 0xFF;
    
    LOG_INF("SET_INTERFACE: interface=%d, alternate=%d", interface_number, alternate_setting);
    
    /* Set interface in device core */
    int ret = usb_device_core_set_interface(dev, interface_number, alternate_setting);
    if (ret) {
        LOG_ERR("Failed to set interface: %d", ret);
        return ret;
    }
    
    /* Send status stage */
    return usb_function_driver_submit_transfer(dev, 0x80, NULL, 0, NULL, NULL);
}

/**
 * @brief Handle SYNCH_FRAME request
 */
int usb_adaptation_handle_synch_frame(struct usb_stack_device *dev,
                                     const struct usb_setup_packet *setup)
{
    /* SYNCH_FRAME is only for isochronous endpoints */
    LOG_WRN("SYNCH_FRAME not implemented");
    return -ENOTSUP;
}
