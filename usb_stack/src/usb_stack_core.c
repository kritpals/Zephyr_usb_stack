/*
 * USB Stack Core Implementation
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "../include/usb_stack.h"

/* USB GET_STATUS response bits for endpoints */
#define USB_GET_STATUS_ENDPOINT_HALT    BIT(0)

LOG_MODULE_REGISTER(usb_stack_core, CONFIG_USB_STACK_LOG_LEVEL);

/* Global USB stack instance */
static struct usb_stack_device *g_usb_stack_device = NULL;

/* Event handling work queue */
K_THREAD_STACK_DEFINE(usb_stack_work_stack, USB_STACK_THREAD_STACK_SIZE);
static struct k_work_q usb_stack_work_q;

/* Event work handler */
static void usb_stack_event_work_handler(struct k_work *work)
{
    struct usb_stack_device *dev = CONTAINER_OF(work, struct usb_stack_device, event_work);
    usb_stack_event_type_t event;
    
    while (k_msgq_get(&dev->event_queue, &event, K_NO_WAIT) == 0) {
        LOG_DBG("Processing USB event: %d", event);
        
        switch (event) {
        case USB_STACK_EVENT_RESET:
            dev->state = USB_STACK_STATE_DEFAULT;
            dev->address = 0;
            dev->configuration = 0;
            dev->reset_count++;
            break;
            
        case USB_STACK_EVENT_SUSPEND:
            if (dev->state != USB_STACK_STATE_SUSPENDED) {
                dev->state = USB_STACK_STATE_SUSPENDED;
                dev->suspend_count++;
            }
            break;
            
        case USB_STACK_EVENT_RESUME:
            if (dev->state == USB_STACK_STATE_SUSPENDED) {
                dev->state = (dev->configuration > 0) ? 
                            USB_STACK_STATE_CONFIGURED : 
                            (dev->address > 0) ? 
                            USB_STACK_STATE_ADDRESS : 
                            USB_STACK_STATE_DEFAULT;
                dev->resume_count++;
            }
            break;
            
        case USB_STACK_EVENT_CONNECT:
            dev->state = USB_STACK_STATE_ATTACHED;
            break;
            
        case USB_STACK_EVENT_DISCONNECT:
            dev->state = USB_STACK_STATE_DETACHED;
            dev->address = 0;
            dev->configuration = 0;
            break;
            
        case USB_STACK_EVENT_SETUP:
            dev->setup_count++;
            /* Setup packet handling is done in interrupt context */
            break;
            
        case USB_STACK_EVENT_TRANSFER_COMPLETE:
            dev->transfer_count++;
            /* Transfer completion is handled by individual transfer callbacks */
            break;
            
        case USB_STACK_EVENT_SOF:
            /* Start of Frame - typically used for isochronous transfers */
            break;
            
        case USB_STACK_EVENT_ERROR:
            dev->error_count++;
            LOG_ERR("USB error event occurred");
            break;
            
        default:
            LOG_WRN("Unknown USB event: %d", event);
            break;
        }
        
        /* Notify application callback if registered */
        if (dev->config && dev->config->event_callback) {
            dev->config->event_callback(dev, event, NULL);
        }
    }
}

/* Submit event to work queue */
static int usb_stack_submit_event(struct usb_stack_device *dev, usb_stack_event_type_t event)
{
    int ret;
    
    ret = k_msgq_put(&dev->event_queue, &event, K_NO_WAIT);
    if (ret) {
        LOG_ERR("Failed to queue USB event %d: %d", event, ret);
        return ret;
    }
    
    k_work_submit_to_queue(&usb_stack_work_q, &dev->event_work);
    return 0;
}

/* Standard device request handlers */
static int handle_get_status(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    uint16_t status = 0;
    
    switch (setup->RequestType.recipient) {
    case USB_REQTYPE_RECIPIENT_DEVICE:
        if (dev->config->self_powered) {
            status |= USB_GET_STATUS_SELF_POWERED;
        }
        if (dev->config->remote_wakeup) {
            status |= USB_GET_STATUS_REMOTE_WAKEUP;
        }
        break;
        
    case USB_REQTYPE_RECIPIENT_INTERFACE:
        /* Interface status is always 0 */
        status = 0;
        break;
        
    case USB_REQTYPE_RECIPIENT_ENDPOINT:
        {
            uint8_t ep_addr = setup->wIndex & 0xFF;
            struct usb_stack_endpoint *ep = usb_stack_get_endpoint(dev, ep_addr);
            if (ep && ep->stalled) {
                status |= USB_GET_STATUS_ENDPOINT_HALT;
            }
        }
        break;
        
    default:
        return -EINVAL;
    }
    
    return usb_stack_control_response(dev, (uint8_t *)&status, sizeof(status));
}

static int handle_set_address(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    uint8_t address = setup->wValue & 0x7F;
    
    LOG_DBG("Set address: %d", address);
    
    /* Address will be set after status stage */
    dev->address = address;
    dev->state = (address == 0) ? USB_STACK_STATE_DEFAULT : USB_STACK_STATE_ADDRESS;
    
    /* Send status response first, then set address in controller */
    int ret = usb_stack_control_response(dev, NULL, 0);
    if (ret == 0 && dev->controller_dev) {
        const struct usb_stack_controller_api *api = dev->controller_dev->api;
        if (api->set_address) {
            api->set_address(dev->controller_dev, address);
        }
    }
    
    return ret;
}

static int handle_get_descriptor(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    uint8_t desc_type = (setup->wValue >> 8) & 0xFF;
    uint8_t desc_index = setup->wValue & 0xFF;
    uint16_t lang_id = setup->wIndex;
    
    LOG_DBG("Get descriptor: type=%d, index=%d, lang=%d", desc_type, desc_index, lang_id);
    
    switch (desc_type) {
    case USB_DESC_DEVICE:
        if (dev->config->device_desc) {
            uint16_t len = MIN(setup->wLength, dev->config->device_desc->bLength);
            return usb_stack_control_response(dev, (uint8_t *)dev->config->device_desc, len);
        }
        break;
        
    case USB_DESC_CONFIGURATION:
        if (dev->config->config_desc) {
            uint16_t len = MIN(setup->wLength, sys_le16_to_cpu(dev->config->config_desc->wTotalLength));
            return usb_stack_control_response(dev, (uint8_t *)dev->config->config_desc, len);
        }
        break;
        
    case USB_DESC_STRING:
        if (desc_index < dev->config->num_strings && dev->config->string_descs[desc_index]) {
            struct usb_string_descriptor *str_desc = dev->config->string_descs[desc_index];
            uint16_t len = MIN(setup->wLength, str_desc->bLength);
            return usb_stack_control_response(dev, (uint8_t *)str_desc, len);
        }
        break;
        
    default:
        LOG_WRN("Unsupported descriptor type: %d", desc_type);
        break;
    }
    
    return -EINVAL;
}

static int handle_set_configuration(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    uint8_t config_value = setup->wValue & 0xFF;
    
    LOG_INF("Set configuration: %d", config_value);
    
    if (config_value == 0) {
        /* Unconfigure device */
        dev->configuration = 0;
        dev->state = (dev->address == 0) ? USB_STACK_STATE_DEFAULT : USB_STACK_STATE_ADDRESS;
        
        /* Deconfigure all non-control endpoints */
        for (int i = 0; i < ARRAY_SIZE(dev->endpoints); i++) {
            struct usb_stack_endpoint *ep = &dev->endpoints[i];
            if (ep->enabled && USB_STACK_EP_NUM(ep->address) != 0) {
                usb_stack_deconfigure_endpoint(dev, ep->address);
            }
        }
    } else if (config_value == 1) {
        /* Configure device */
        dev->configuration = config_value;
        dev->state = USB_STACK_STATE_CONFIGURED;
        
        /* Configure endpoints based on configuration descriptor */
        /* This would typically parse the configuration descriptor */
        /* For now, we'll configure a simple bulk endpoint pair */
        usb_stack_configure_endpoint(dev, 0x81, USB_STACK_EP_TYPE_BULK, 512, 0);  /* Bulk IN */
        usb_stack_configure_endpoint(dev, 0x01, USB_STACK_EP_TYPE_BULK, 512, 0);  /* Bulk OUT */
    } else {
        return -EINVAL;
    }
    
    return usb_stack_control_response(dev, NULL, 0);
}

static int handle_get_configuration(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    return usb_stack_control_response(dev, &dev->configuration, 1);
}

/* Standard setup packet handler */
static int usb_stack_handle_standard_setup(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    switch (setup->bRequest) {
    case USB_SREQ_GET_STATUS:
        return handle_get_status(dev, setup);
        
    case USB_SREQ_SET_ADDRESS:
        return handle_set_address(dev, setup);
        
    case USB_SREQ_GET_DESCRIPTOR:
        return handle_get_descriptor(dev, setup);
        
    case USB_SREQ_SET_CONFIGURATION:
        return handle_set_configuration(dev, setup);
        
    case USB_SREQ_GET_CONFIGURATION:
        return handle_get_configuration(dev, setup);
        
    case USB_SREQ_SET_FEATURE:
    case USB_SREQ_CLEAR_FEATURE:
        /* Feature handling for endpoints, interfaces, device */
        if (setup->RequestType.recipient == USB_REQTYPE_RECIPIENT_ENDPOINT) {
            uint8_t ep_addr = setup->wIndex & 0xFF;
            if (setup->wValue == USB_SFS_ENDPOINT_HALT) {
                if (setup->bRequest == USB_SREQ_SET_FEATURE) {
                    return usb_stack_stall_endpoint(dev, ep_addr);
                } else {
                    return usb_stack_unstall_endpoint(dev, ep_addr);
                }
            }
        }
        return usb_stack_control_response(dev, NULL, 0);
        
    default:
        LOG_WRN("Unsupported standard request: 0x%02x", setup->bRequest);
        return -EINVAL;
    }
}

/* Setup packet processing */
static int usb_stack_process_setup(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    int ret = -EINVAL;
    
    LOG_DBG("Setup: bmRequestType=0x%02x, bRequest=0x%02x, wValue=0x%04x, wIndex=0x%04x, wLength=%d",
            setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);
    
    /* Copy setup packet for reference */
    memcpy(&dev->setup_packet, setup, sizeof(*setup));
    
    switch (setup->RequestType.type) {
    case USB_REQTYPE_TYPE_STANDARD:
        ret = usb_stack_handle_standard_setup(dev, setup);
        break;
        
    case USB_REQTYPE_TYPE_CLASS:
    case USB_REQTYPE_TYPE_VENDOR:
        /* Forward to application setup callback */
        if (dev->config && dev->config->setup_callback) {
            ret = dev->config->setup_callback(dev, setup);
        }
        break;
        
    default:
        LOG_WRN("Unknown request type: %d", setup->RequestType.type);
        break;
    }
    
    if (ret < 0) {
        LOG_WRN("Setup request failed: %d", ret);
        usb_stack_control_stall(dev);
    }
    
    return ret;
}

/* USB Stack API Implementation */

int usb_stack_init(struct usb_stack_device *dev, const struct usb_stack_device_config *config)
{
    if (!dev || !config) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    LOG_INF("Initializing USB stack");
    
    /* Initialize device structure */
    memset(dev, 0, sizeof(*dev));
    dev->config = config;
    dev->state = USB_STACK_STATE_DETACHED;
    dev->speed = USB_STACK_SPEED_UNKNOWN;
    
    /* Initialize synchronization objects */
    k_mutex_init(&dev->device_mutex);
    k_sem_init(&dev->init_sem, 0, 1);
    
    /* Initialize event handling */
    k_work_init(&dev->event_work, usb_stack_event_work_handler);
    k_msgq_init(&dev->event_queue, dev->event_buffer, sizeof(usb_stack_event_type_t), 16);
    
    /* Initialize endpoints */
    for (int i = 0; i < ARRAY_SIZE(dev->endpoints); i++) {
        struct usb_stack_endpoint *ep = &dev->endpoints[i];
        ep->address = 0xFF;  /* Invalid address */
        ep->enabled = false;
        ep->stalled = false;
        sys_dlist_init(&ep->transfer_queue);
        k_mutex_init(&ep->queue_mutex);
    }
    
    /* Initialize work queue */
    k_work_queue_init(&usb_stack_work_q);
    k_work_queue_start(&usb_stack_work_q, usb_stack_work_stack,
                      K_THREAD_STACK_SIZEOF(usb_stack_work_stack),
                      USB_STACK_THREAD_PRIORITY, NULL);
    
    /* Set global instance */
    g_usb_stack_device = dev;
    
    LOG_INF("USB stack initialized successfully");
    return USB_STACK_SUCCESS;
}

int usb_stack_deinit(struct usb_stack_device *dev)
{
    if (!dev) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    LOG_INF("Deinitializing USB stack");
    
    /* Disable device */
    usb_stack_disable(dev);
    
    /* Cancel all work */
    k_work_cancel(&dev->event_work);
    
    /* Reset state */
    dev->state = USB_STACK_STATE_DETACHED;
    dev->speed = USB_STACK_SPEED_UNKNOWN;
    dev->address = 0;
    dev->configuration = 0;
    
    /* Clear global instance */
    if (g_usb_stack_device == dev) {
        g_usb_stack_device = NULL;
    }
    
    return USB_STACK_SUCCESS;
}

int usb_stack_enable(struct usb_stack_device *dev)
{
    if (!dev) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    LOG_INF("Enabling USB stack");
    
    k_mutex_lock(&dev->device_mutex, K_FOREVER);
    
    /* Initialize PHY if available */
    if (dev->phy_dev) {
        const struct usb_stack_phy_api *phy_api = dev->phy_dev->api;
        if (phy_api->init) {
            int ret = phy_api->init(dev->phy_dev);
            if (ret) {
                LOG_ERR("PHY initialization failed: %d", ret);
                k_mutex_unlock(&dev->device_mutex);
                return ret;
            }
        }
        if (phy_api->power_on) {
            int ret = phy_api->power_on(dev->phy_dev);
            if (ret) {
                LOG_ERR("PHY power on failed: %d", ret);
                k_mutex_unlock(&dev->device_mutex);
                return ret;
            }
        }
    }
    
    /* Initialize controller */
    if (dev->controller_dev) {
        const struct usb_stack_controller_api *ctrl_api = dev->controller_dev->api;
        if (ctrl_api->init) {
            int ret = ctrl_api->init(dev->controller_dev);
            if (ret) {
                LOG_ERR("Controller initialization failed: %d", ret);
                k_mutex_unlock(&dev->device_mutex);
                return ret;
            }
        }
        if (ctrl_api->enable) {
            int ret = ctrl_api->enable(dev->controller_dev);
            if (ret) {
                LOG_ERR("Controller enable failed: %d", ret);
                k_mutex_unlock(&dev->device_mutex);
                return ret;
            }
        }
    }
    
    /* Enable Type-C manager if available */
    if (dev->typec_dev) {
        const struct usb_stack_typec_api *typec_api = dev->typec_dev->api;
        if (typec_api->enable) {
            int ret = typec_api->enable(dev->typec_dev);
            if (ret) {
                LOG_ERR("Type-C enable failed: %d", ret);
                k_mutex_unlock(&dev->device_mutex);
                return ret;
            }
        }
    }
    
    dev->state = USB_STACK_STATE_POWERED;
    
    k_mutex_unlock(&dev->device_mutex);
    
    LOG_INF("USB stack enabled successfully");
    return USB_STACK_SUCCESS;
}

int usb_stack_disable(struct usb_stack_device *dev)
{
    if (!dev) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    LOG_INF("Disabling USB stack");
    
    k_mutex_lock(&dev->device_mutex, K_FOREVER);
    
    /* Disable controller */
    if (dev->controller_dev) {
        const struct usb_stack_controller_api *ctrl_api = dev->controller_dev->api;
        if (ctrl_api->disable) {
            ctrl_api->disable(dev->controller_dev);
        }
    }
    
    /* Disable PHY */
    if (dev->phy_dev) {
        const struct usb_stack_phy_api *phy_api = dev->phy_dev->api;
        if (phy_api->power_off) {
            phy_api->power_off(dev->phy_dev);
        }
    }
    
    /* Disable Type-C manager */
    if (dev->typec_dev) {
        const struct usb_stack_typec_api *typec_api = dev->typec_dev->api;
        if (typec_api->disable) {
            typec_api->disable(dev->typec_dev);
        }
    }
    
    dev->state = USB_STACK_STATE_DETACHED;
    
    k_mutex_unlock(&dev->device_mutex);
    
    return USB_STACK_SUCCESS;
}

/* Endpoint management */
int usb_stack_configure_endpoint(struct usb_stack_device *dev, uint8_t ep_addr,
                                usb_stack_ep_type_t type, uint16_t max_packet_size, uint8_t interval)
{
    if (!dev) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    if (ep_index >= ARRAY_SIZE(dev->endpoints)) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    
    k_mutex_lock(&ep->queue_mutex, K_FOREVER);
    
    ep->address = ep_addr;
    ep->type = type;
    ep->max_packet_size = max_packet_size;
    ep->interval = interval;
    ep->enabled = true;
    ep->stalled = false;
    
    k_mutex_unlock(&ep->queue_mutex);
    
    /* Configure in controller */
    if (dev->controller_dev) {
        const struct usb_stack_controller_api *api = dev->controller_dev->api;
        if (api->configure_endpoint) {
            return api->configure_endpoint(dev->controller_dev, ep);
        }
    }
    
    LOG_DBG("Configured endpoint 0x%02x (type=%d, mps=%d)", ep_addr, type, max_packet_size);
    return USB_STACK_SUCCESS;
}

struct usb_stack_endpoint *usb_stack_get_endpoint(struct usb_stack_device *dev, uint8_t ep_addr)
{
    if (!dev) {
        return NULL;
    }
    
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    if (ep_index >= ARRAY_SIZE(dev->endpoints)) {
        return NULL;
    }
    
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    return (ep->enabled && ep->address == ep_addr) ? ep : NULL;
}

/* Control transfer support */
int usb_stack_control_response(struct usb_stack_device *dev, uint8_t *buffer, uint32_t length)
{
    if (!dev) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    /* Submit control IN transfer */
    struct usb_stack_transfer transfer;
    usb_stack_init_transfer(&transfer, 0x80, buffer, length, NULL, NULL);
    
    return usb_stack_submit_transfer(dev, &transfer);
}

int usb_stack_control_stall(struct usb_stack_device *dev)
{
    if (!dev) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    /* Stall both control endpoints */
    usb_stack_stall_endpoint(dev, 0x00);
    usb_stack_stall_endpoint(dev, 0x80);
    
    return USB_STACK_SUCCESS;
}

/* Transfer management */
void usb_stack_init_transfer(struct usb_stack_transfer *transfer, uint8_t ep_addr,
                            uint8_t *buffer, uint32_t length,
                            usb_stack_transfer_callback_t callback, void *user_data)
{
    if (!transfer) {
        return;
    }
    
    memset(transfer, 0, sizeof(*transfer));
    transfer->endpoint = ep_addr;
    transfer->buffer = buffer;
    transfer->length = length;
    transfer->callback = callback;
    transfer->user_data = user_data;
    transfer->status = USB_STACK_TRANSFER_IDLE;
    
    k_sem_init(&transfer->completion_sem, 0, 1);
}

int usb_stack_submit_transfer(struct usb_stack_device *dev, struct usb_stack_transfer *transfer)
{
    if (!dev || !transfer) {
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    if (dev->controller_dev) {
        const struct usb_stack_controller_api *api = dev->controller_dev->api;
        if (api->submit_transfer) {
            return api->submit_transfer(dev->controller_dev, transfer);
        }
    }
    
    return USB_STACK_ERROR_NOT_SUPPORTED;
}

/* Getter functions */
usb_stack_device_state_t usb_stack_get_state(struct usb_stack_device *dev)
{
    return dev ? dev->state : USB_STACK_STATE_DETACHED;
}

usb_stack_speed_t usb_stack_get_speed(struct usb_stack_device *dev)
{
    if (!dev) {
        return USB_STACK_SPEED_UNKNOWN;
    }
    
    if (dev->controller_dev) {
        const struct usb_stack_controller_api *api = dev->controller_dev->api;
        if (api->get_speed) {
            return api->get_speed(dev->controller_dev);
        }
    }
    
    return dev->speed;
}

uint8_t usb_stack_get_address(struct usb_stack_device *dev)
{
    return dev ? dev->address : 0;
}

uint8_t usb_stack_get_configuration(struct usb_stack_device *dev)
{
    return dev ? dev->configuration : 0;
}

/* Statistics */
void usb_stack_get_statistics(struct usb_stack_device *dev,
                             uint32_t *reset_count, uint32_t *suspend_count,
                             uint32_t *resume_count, uint32_t *setup_count,
                             uint32_t *transfer_count, uint32_t *error_count)
{
    if (!dev) {
        return;
    }
    
    if (reset_count) *reset_count = dev->reset_count;
    if (suspend_count) *suspend_count = dev->suspend_count;
    if (resume_count) *resume_count = dev->resume_count;
    if (setup_count) *setup_count = dev->setup_count;
    if (transfer_count) *transfer_count = dev->transfer_count;
    if (error_count) *error_count = dev->error_count;
}

void usb_stack_reset_statistics(struct usb_stack_device *dev)
{
    if (!dev) {
        return;
    }
    
    dev->reset_count = 0;
    dev->suspend_count = 0;
    dev->resume_count = 0;
    dev->setup_count = 0;
    dev->transfer_count = 0;
    dev->error_count = 0;
}

/* Global event submission function for use by drivers */
int usb_stack_submit_global_event(usb_stack_event_type_t event)
{
    if (g_usb_stack_device) {
        return usb_stack_submit_event(g_usb_stack_device, event);
    }
    return USB_STACK_ERROR_INVALID_PARAM;
}

/* Global setup processing function for use by drivers */
int usb_stack_process_global_setup(struct usb_setup_packet *setup)
{
    if (g_usb_stack_device && setup) {
        return usb_stack_process_setup(g_usb_stack_device, setup);
    }
    return USB_STACK_ERROR_INVALID_PARAM;
}
