/*
 * USB Adaptation Layer
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "../include/usb_stack.h"
#include "../include/usb_stack_types.h"

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
    
    /* Initialize adaptation layer state */
    dev->adaptation.num_interfaces = 0;
    dev->adaptation.current_config = 0;
    
    /* Initialize interface descriptors */
    for (int i = 0; i < USB_STACK_MAX_INTERFACES; i++) {
        dev->adaptation.interfaces[i].interface_number = 0;
        dev->adaptation.interfaces[i].alternate_setting = 0;
        dev->adaptation.interfaces[i].num_endpoints = 0;
        dev->adaptation.interfaces[i].class_code = 0;
        dev->adaptation.interfaces[i].subclass_code = 0;
        dev->adaptation.interfaces[i].protocol_code = 0;
        dev->adaptation.interfaces[i].interface_string = 0;
        dev->adaptation.interfaces[i].class_handler = NULL;
        dev->adaptation.interfaces[i].user_data = NULL;
    }
    
    /* Initialize class handlers */
    for (int i = 0; i < USB_STACK_MAX_CLASS_HANDLERS; i++) {
        dev->adaptation.class_handlers[i].class_code = 0;
        dev->adaptation.class_handlers[i].handler = NULL;
        dev->adaptation.class_handlers[i].user_data = NULL;
    }
    
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
    
    /* Reset adaptation layer state */
    memset(&dev->adaptation, 0, sizeof(dev->adaptation));
    
    LOG_DBG("USB adaptation layer deinitialized");
    return 0;
}

/**
 * @brief Register a USB class handler
 */
int usb_adaptation_register_class_handler(struct usb_stack_device *dev,
                                         uint8_t class_code,
                                         usb_class_handler_t handler,
                                         void *user_data)
{
    if (!dev || !handler) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    /* Find free slot */
    for (int i = 0; i < USB_STACK_MAX_CLASS_HANDLERS; i++) {
        if (dev->adaptation.class_handlers[i].handler == NULL) {
            dev->adaptation.class_handlers[i].class_code = class_code;
            dev->adaptation.class_handlers[i].handler = handler;
            dev->adaptation.class_handlers[i].user_data = user_data;
            
            LOG_INF("Registered class handler for class 0x%02x", class_code);
            return 0;
        }
    }
    
    LOG_ERR("No free class handler slots available");
    return -ENOMEM;
}

/**
 * @brief Unregister a USB class handler
 */
int usb_adaptation_unregister_class_handler(struct usb_stack_device *dev, uint8_t class_code)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    /* Find and remove handler */
    for (int i = 0; i < USB_STACK_MAX_CLASS_HANDLERS; i++) {
        if (dev->adaptation.class_handlers[i].class_code == class_code) {
            memset(&dev->adaptation.class_handlers[i], 0, sizeof(dev->adaptation.class_handlers[i]));
            LOG_INF("Unregistered class handler for class 0x%02x", class_code);
            return 0;
        }
    }
    
    LOG_WRN("Class handler for class 0x%02x not found", class_code);
    return -ENOENT;
}

/**
 * @brief Add a USB interface
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
    
    if (dev->adaptation.num_interfaces >= USB_STACK_MAX_INTERFACES) {
        LOG_ERR("Maximum number of interfaces reached");
        return -ENOMEM;
    }
    
    struct usb_interface_descriptor *iface = &dev->adaptation.interfaces[dev->adaptation.num_interfaces];
    
    iface->interface_number = interface_number;
    iface->alternate_setting = alternate_setting;
    iface->num_endpoints = 0;
    iface->class_code = class_code;
    iface->subclass_code = subclass_code;
    iface->protocol_code = protocol_code;
    iface->interface_string = interface_string;
    
    /* Find class handler */
    for (int i = 0; i < USB_STACK_MAX_CLASS_HANDLERS; i++) {
        if (dev->adaptation.class_handlers[i].class_code == class_code) {
            iface->class_handler = dev->adaptation.class_handlers[i].handler;
            iface->user_data = dev->adaptation.class_handlers[i].user_data;
            break;
        }
    }
    
    dev->adaptation.num_interfaces++;
    
    LOG_INF("Added interface %d: class=0x%02x, subclass=0x%02x, protocol=0x%02x",
            interface_number, class_code, subclass_code, protocol_code);
    
    return 0;
}

/**
 * @brief Remove a USB interface
 */
int usb_adaptation_remove_interface(struct usb_stack_device *dev, uint8_t interface_number)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    /* Find and remove interface */
    for (int i = 0; i < dev->adaptation.num_interfaces; i++) {
        if (dev->adaptation.interfaces[i].interface_number == interface_number) {
            /* Shift remaining interfaces */
            for (int j = i; j < dev->adaptation.num_interfaces - 1; j++) {
                dev->adaptation.interfaces[j] = dev->adaptation.interfaces[j + 1];
            }
            
            dev->adaptation.num_interfaces--;
            memset(&dev->adaptation.interfaces[dev->adaptation.num_interfaces], 0, 
                   sizeof(dev->adaptation.interfaces[dev->adaptation.num_interfaces]));
            
            LOG_INF("Removed interface %d", interface_number);
            return 0;
        }
    }
    
    LOG_WRN("Interface %d not found", interface_number);
    return -ENOENT;
}

/**
 * @brief Get interface descriptor
 */
const struct usb_interface_descriptor *usb_adaptation_get_interface(struct usb_stack_device *dev,
                                                                   uint8_t interface_number)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return NULL;
    }
    
    for (int i = 0; i < dev->adaptation.num_interfaces; i++) {
        if (dev->adaptation.interfaces[i].interface_number == interface_number) {
            return &dev->adaptation.interfaces[i];
        }
    }
    
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
        
        /* Find interface */
        const struct usb_interface_descriptor *iface = usb_adaptation_get_interface(dev, interface_number);
        if (iface && iface->class_handler) {
            /* Forward to class handler */
            return iface->class_handler(dev, setup, iface->user_data);
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
        if (dev->config.pm_support) {
            status_data |= USB_STATUS_SELF_POWERED;
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
    
    /* Set address in hardware */
    int ret = dwc3_controller_set_address(&dev->controller, address);
    if (ret) {
        LOG_ERR("Failed to set address in hardware: %d", ret);
        return ret;
    }
    
    /* Update device state */
    if (address == 0) {
        usb_device_core_set_state(dev, USB_DEVICE_STATE_DEFAULT);
    } else {
        usb_device_core_set_state(dev, USB_DEVICE_STATE_ADDRESS);
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
    
    dev->adaptation.current_config = config;
    
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
