/*
 * USB Function Driver Layer
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "../include/usb_stack.h"
#include "../include/usb_stack_types.h"

LOG_MODULE_REGISTER(usb_function_driver, CONFIG_USB_STACK_LOG_LEVEL);

/**
 * @brief Initialize USB function driver
 */
int usb_function_driver_init(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Initializing USB function driver");
    
    /* Initialize device mutex and semaphore if not already done */
    k_mutex_init(&dev->device_mutex);
    k_sem_init(&dev->init_sem, 0, 1);
    
    /* Initialize event work and queue if not already done */
    k_work_init(&dev->event_work, NULL);  /* Will be set by event handler */
    k_msgq_init(&dev->event_queue, dev->event_buffer, sizeof(usb_stack_event_type_t), 16);
    
    /* Reset statistics */
    dev->reset_count = 0;
    dev->suspend_count = 0;
    dev->resume_count = 0;
    dev->setup_count = 0;
    dev->transfer_count = 0;
    dev->error_count = 0;
    
    LOG_DBG("USB function driver initialized successfully");
    return 0;
}

/**
 * @brief Deinitialize USB function driver
 */
int usb_function_driver_deinit(struct usb_stack_device *dev)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    LOG_INF("Deinitializing USB function driver");
    
    /* Cancel all active transfers on all endpoints */
    for (int i = 0; i < USB_STACK_MAX_ENDPOINTS * 2; i++) {
        struct usb_stack_endpoint *ep = &dev->endpoints[i];
        if (ep->enabled) {
            /* Cancel any pending transfers on this endpoint */
            sys_dnode_t *node, *next;
            SYS_DLIST_FOR_EACH_NODE_SAFE(&ep->transfer_queue, node, next) {
                struct usb_stack_transfer *transfer = CONTAINER_OF(node, struct usb_stack_transfer, node);
                transfer->status = USB_STACK_TRANSFER_CANCELLED;
                sys_dlist_remove(node);
            }
        }
    }
    
    /* Reset statistics */
    dev->transfer_count = 0;
    dev->error_count = 0;
    
    LOG_DBG("USB function driver deinitialized");
    return 0;
}

/**
 * @brief Allocate a transfer from the stack
 */
static struct usb_stack_transfer *allocate_transfer(struct usb_stack_device *dev)
{
    /* Use the stack's built-in transfer allocation */
    struct usb_stack_transfer *transfer = k_malloc(sizeof(struct usb_stack_transfer));
    if (!transfer) {
        LOG_WRN("No memory for transfer allocation");
        return NULL;
    }
    
    memset(transfer, 0, sizeof(*transfer));
    k_sem_init(&transfer->completion_sem, 0, 1);
    transfer->status = USB_STACK_TRANSFER_IDLE;
    
    return transfer;
}

/**
 * @brief Free a transfer back to the system
 */
static void free_transfer(struct usb_stack_device *dev, struct usb_stack_transfer *transfer)
{
    if (!transfer) {
        return;
    }
    
    k_free(transfer);
}

/**
 * @brief Submit a USB transfer
 */
int usb_function_driver_submit_transfer(struct usb_stack_device *dev,
                                       uint8_t ep_addr,
                                       uint8_t *buffer,
                                       size_t length,
                                       usb_stack_transfer_callback_t callback,
                                       void *user_data)
{
    if (!dev || !buffer) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    if (ep_index >= USB_STACK_MAX_ENDPOINTS * 2) {
        LOG_ERR("Invalid endpoint index: %d", ep_index);
        return -EINVAL;
    }
    
    /* Allocate transfer */
    struct usb_stack_transfer *transfer = allocate_transfer(dev);
    if (!transfer) {
        LOG_ERR("Failed to allocate transfer");
        return -ENOMEM;
    }
    
    /* Initialize transfer */
    transfer->endpoint = ep_addr;
    transfer->buffer = buffer;
    transfer->length = length;
    transfer->actual_length = 0;
    transfer->status = USB_STACK_TRANSFER_PENDING;
    transfer->callback = callback;
    transfer->user_data = user_data;
    
    LOG_DBG("Submitting transfer: EP 0x%02x, length=%d", ep_addr, length);
    
    /* Add to endpoint queue */
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    k_mutex_lock(&ep->queue_mutex, K_FOREVER);
    sys_dlist_append(&ep->transfer_queue, &transfer->node);
    k_mutex_unlock(&ep->queue_mutex);
    
    /* Submit to USB stack */
    int ret = usb_stack_submit_transfer(dev, transfer);
    if (ret) {
        LOG_ERR("Failed to submit transfer to USB stack: %d", ret);
        k_mutex_lock(&ep->queue_mutex, K_FOREVER);
        sys_dlist_remove(&transfer->node);
        k_mutex_unlock(&ep->queue_mutex);
        free_transfer(dev, transfer);
        return ret;
    }
    
    dev->transfer_count++;
    return 0;
}

/**
 * @brief Cancel a USB transfer
 */
int usb_function_driver_cancel_transfer(struct usb_stack_device *dev, struct usb_stack_transfer *transfer)
{
    if (!dev || !transfer) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    if (transfer->status == USB_STACK_TRANSFER_IDLE) {
        LOG_WRN("Transfer not active");
        return -EINVAL;
    }
    
    LOG_DBG("Cancelling transfer: EP 0x%02x", transfer->endpoint);
    
    /* Cancel in USB stack */
    int ret = usb_stack_cancel_transfer(dev, transfer);
    if (ret) {
        LOG_WRN("Failed to cancel transfer in USB stack: %d", ret);
    }
    
    /* Remove from queue */
    uint8_t ep_index = USB_STACK_EP_INDEX(transfer->endpoint);
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    k_mutex_lock(&ep->queue_mutex, K_FOREVER);
    sys_dlist_remove(&transfer->node);
    k_mutex_unlock(&ep->queue_mutex);
    
    /* Update status and call callback */
    transfer->status = USB_STACK_TRANSFER_CANCELLED;
    if (transfer->callback) {
        transfer->callback(transfer);
    }
    
    /* Free transfer */
    free_transfer(dev, transfer);
    
    return 0;
}

/**
 * @brief Handle transfer completion (called from hardware layer)
 */
void usb_function_driver_transfer_complete(struct usb_stack_device *dev,
                                          struct usb_stack_transfer *transfer,
                                          usb_stack_transfer_status_t status,
                                          size_t transferred)
{
    if (!dev || !transfer) {
        LOG_ERR("Invalid parameters");
        return;
    }
    
    LOG_DBG("Transfer completed: EP 0x%02x, status=%d, transferred=%d",
            transfer->endpoint, status, transferred);
    
    /* Update transfer status */
    transfer->status = status;
    transfer->actual_length = transferred;
    
    /* Call completion callback */
    if (transfer->callback) {
        transfer->callback(transfer);
    }
    
    /* Update statistics */
    if (status == USB_STACK_TRANSFER_COMPLETE) {
        dev->transfer_count++;
    } else {
        dev->error_count++;
    }
}

/**
 * @brief Get transfer statistics
 */
int usb_function_driver_get_statistics(struct usb_stack_device *dev,
                                      uint32_t *active_transfers,
                                      uint32_t *completed_transfers,
                                      uint32_t *failed_transfers)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return -EINVAL;
    }
    
    if (active_transfers) {
        /* Count active transfers across all endpoints */
        uint32_t count = 0;
        for (int i = 0; i < USB_STACK_MAX_ENDPOINTS * 2; i++) {
            struct usb_stack_endpoint *ep = &dev->endpoints[i];
            if (ep->enabled) {
                sys_dnode_t *node;
                SYS_DLIST_FOR_EACH_NODE(&ep->transfer_queue, node) {
                    count++;
                }
            }
        }
        *active_transfers = count;
    }
    
    if (completed_transfers) {
        *completed_transfers = dev->transfer_count;
    }
    
    if (failed_transfers) {
        *failed_transfers = dev->error_count;
    }
    
    return 0;
}

/**
 * @brief Setup control transfer
 */
int usb_function_driver_setup_control_transfer(struct usb_stack_device *dev,
                                              const struct usb_setup_packet *setup,
                                              uint8_t *data_buffer,
                                              size_t data_length,
                                              usb_stack_transfer_callback_t callback,
                                              void *user_data)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    LOG_DBG("Setting up control transfer: bmRequestType=0x%02x, bRequest=0x%02x, wValue=0x%04x, wIndex=0x%04x, wLength=%d",
            setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);
    
    /* Store setup packet */
    memcpy(&dev->setup_packet, setup, sizeof(*setup));
    dev->setup_count++;
    
    /* Allocate transfer for data stage (if needed) */
    if (data_length > 0 && data_buffer) {
        uint8_t ep_addr = (setup->bmRequestType & 0x80) ? 0x80 : 0x00;
        return usb_function_driver_submit_transfer(dev, ep_addr, data_buffer, data_length, callback, user_data);
    }
    
    /* Status stage only */
    uint8_t status_ep = (setup->bmRequestType & 0x80) ? 0x00 : 0x80;
    return usb_function_driver_submit_transfer(dev, status_ep, NULL, 0, callback, user_data);
}

/**
 * @brief Stall endpoint
 */
int usb_function_driver_stall_endpoint(struct usb_stack_device *dev, uint8_t ep_addr)
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
    
    LOG_DBG("Stalling endpoint 0x%02x", ep_addr);
    
    /* Mark endpoint as stalled */
    dev->endpoints[ep_index].stalled = true;
    
    /* Stall using USB stack */
    return usb_stack_stall_endpoint(dev, ep_addr);
}

/**
 * @brief Clear endpoint stall
 */
int usb_function_driver_clear_stall(struct usb_stack_device *dev, uint8_t ep_addr)
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
    
    LOG_DBG("Clearing stall on endpoint 0x%02x", ep_addr);
    
    /* Clear stall flag */
    dev->endpoints[ep_index].stalled = false;
    
    /* Clear stall using USB stack */
    return usb_stack_unstall_endpoint(dev, ep_addr);
}

/**
 * @brief Check if endpoint is stalled
 */
bool usb_function_driver_is_endpoint_stalled(struct usb_stack_device *dev, uint8_t ep_addr)
{
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return false;
    }
    
    uint8_t ep_index = USB_STACK_EP_INDEX(ep_addr);
    if (ep_index >= USB_STACK_MAX_ENDPOINTS * 2) {
        LOG_ERR("Invalid endpoint index: %d", ep_index);
        return false;
    }
    
    /* Check stall status */
    return dev->endpoints[ep_index].stalled;
}

/**
 * @brief Flush endpoint
 */
int usb_function_driver_flush_endpoint(struct usb_stack_device *dev, uint8_t ep_addr)
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
    
    LOG_DBG("Flushing endpoint 0x%02x", ep_addr);
    
    /* Cancel all pending transfers for this endpoint */
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    k_mutex_lock(&ep->queue_mutex, K_FOREVER);
    
    sys_dnode_t *node, *next;
    SYS_DLIST_FOR_EACH_NODE_SAFE(&ep->transfer_queue, node, next) {
        struct usb_stack_transfer *transfer = CONTAINER_OF(node, struct usb_stack_transfer, node);
        usb_function_driver_cancel_transfer(dev, transfer);
    }
    
    k_mutex_unlock(&ep->queue_mutex);
    
    return 0;
}

/**
 * @brief Get endpoint transfer queue depth
 */
int usb_function_driver_get_queue_depth(struct usb_stack_device *dev, uint8_t ep_addr)
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
    
    int depth = 0;
    struct usb_stack_endpoint *ep = &dev->endpoints[ep_index];
    
    k_mutex_lock(&ep->queue_mutex, K_FOREVER);
    sys_dnode_t *node;
    SYS_DLIST_FOR_EACH_NODE(&ep->transfer_queue, node) {
        depth++;
    }
    k_mutex_unlock(&ep->queue_mutex);
    
    return depth;
}
