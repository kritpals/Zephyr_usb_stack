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
    
    /* Initialize function driver state */
    dev->function_driver.active_transfers = 0;
    dev->function_driver.max_transfers = USB_STACK_MAX_CONCURRENT_TRANSFERS;
    
    /* Initialize transfer pool */
    for (int i = 0; i < USB_STACK_MAX_CONCURRENT_TRANSFERS; i++) {
        dev->function_driver.transfer_pool[i].in_use = false;
        dev->function_driver.transfer_pool[i].ep_addr = 0;
        dev->function_driver.transfer_pool[i].buffer = NULL;
        dev->function_driver.transfer_pool[i].length = 0;
        dev->function_driver.transfer_pool[i].transferred = 0;
        dev->function_driver.transfer_pool[i].status = USB_TRANSFER_STATUS_IDLE;
        dev->function_driver.transfer_pool[i].callback = NULL;
        dev->function_driver.transfer_pool[i].user_data = NULL;
    }
    
    /* Initialize endpoint transfer queues */
    for (int i = 0; i < USB_STACK_MAX_ENDPOINTS; i++) {
        sys_slist_init(&dev->function_driver.ep_transfer_queues[i]);
    }
    
    /* Initialize work queue for transfer processing */
    k_work_queue_init(&dev->function_driver.transfer_work_queue);
    k_work_queue_start(&dev->function_driver.transfer_work_queue,
                      dev->function_driver.transfer_work_stack,
                      K_THREAD_STACK_SIZEOF(dev->function_driver.transfer_work_stack),
                      CONFIG_USB_STACK_THREAD_PRIORITY, NULL);
    
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
    
    /* Cancel all active transfers */
    for (int i = 0; i < USB_STACK_MAX_CONCURRENT_TRANSFERS; i++) {
        if (dev->function_driver.transfer_pool[i].in_use) {
            usb_function_driver_cancel_transfer(dev, &dev->function_driver.transfer_pool[i]);
        }
    }
    
    /* Reset function driver state */
    memset(&dev->function_driver, 0, sizeof(dev->function_driver));
    
    LOG_DBG("USB function driver deinitialized");
    return 0;
}

/**
 * @brief Allocate a transfer from the pool
 */
static struct usb_transfer *allocate_transfer(struct usb_stack_device *dev)
{
    for (int i = 0; i < USB_STACK_MAX_CONCURRENT_TRANSFERS; i++) {
        if (!dev->function_driver.transfer_pool[i].in_use) {
            struct usb_transfer *transfer = &dev->function_driver.transfer_pool[i];
            memset(transfer, 0, sizeof(*transfer));
            transfer->in_use = true;
            dev->function_driver.active_transfers++;
            return transfer;
        }
    }
    
    LOG_WRN("No free transfers available");
    return NULL;
}

/**
 * @brief Free a transfer back to the pool
 */
static void free_transfer(struct usb_stack_device *dev, struct usb_transfer *transfer)
{
    if (!transfer || !transfer->in_use) {
        return;
    }
    
    transfer->in_use = false;
    dev->function_driver.active_transfers--;
}

/**
 * @brief Submit a USB transfer
 */
int usb_function_driver_submit_transfer(struct usb_stack_device *dev,
                                       uint8_t ep_addr,
                                       uint8_t *buffer,
                                       size_t length,
                                       usb_transfer_callback_t callback,
                                       void *user_data)
{
    if (!dev || !buffer) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint8_t ep_num = ep_addr & 0x0F;
    if (ep_num >= USB_STACK_MAX_ENDPOINTS) {
        LOG_ERR("Invalid endpoint number: %d", ep_num);
        return -EINVAL;
    }
    
    /* Allocate transfer */
    struct usb_transfer *transfer = allocate_transfer(dev);
    if (!transfer) {
        LOG_ERR("Failed to allocate transfer");
        return -ENOMEM;
    }
    
    /* Initialize transfer */
    transfer->ep_addr = ep_addr;
    transfer->buffer = buffer;
    transfer->length = length;
    transfer->transferred = 0;
    transfer->status = USB_TRANSFER_STATUS_PENDING;
    transfer->callback = callback;
    transfer->user_data = user_data;
    
    LOG_DBG("Submitting transfer: EP%d %s, length=%d",
            ep_num, (ep_addr & 0x80) ? "IN" : "OUT", length);
    
    /* Add to endpoint queue */
    sys_slist_append(&dev->function_driver.ep_transfer_queues[ep_num], &transfer->node);
    
    /* Submit to hardware */
    int ret = dwc3_controller_submit_transfer(&dev->controller, transfer);
    if (ret) {
        LOG_ERR("Failed to submit transfer to hardware: %d", ret);
        sys_slist_find_and_remove(&dev->function_driver.ep_transfer_queues[ep_num], &transfer->node);
        free_transfer(dev, transfer);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Cancel a USB transfer
 */
int usb_function_driver_cancel_transfer(struct usb_stack_device *dev, struct usb_transfer *transfer)
{
    if (!dev || !transfer) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    if (!transfer->in_use) {
        LOG_WRN("Transfer not in use");
        return -EINVAL;
    }
    
    LOG_DBG("Cancelling transfer: EP%d", transfer->ep_addr & 0x0F);
    
    /* Cancel in hardware */
    int ret = dwc3_controller_cancel_transfer(&dev->controller, transfer);
    if (ret) {
        LOG_WRN("Failed to cancel transfer in hardware: %d", ret);
    }
    
    /* Remove from queue */
    uint8_t ep_num = transfer->ep_addr & 0x0F;
    sys_slist_find_and_remove(&dev->function_driver.ep_transfer_queues[ep_num], &transfer->node);
    
    /* Update status and call callback */
    transfer->status = USB_TRANSFER_STATUS_CANCELLED;
    if (transfer->callback) {
        transfer->callback(transfer, transfer->user_data);
    }
    
    /* Free transfer */
    free_transfer(dev, transfer);
    
    return 0;
}

/**
 * @brief Transfer completion work handler
 */
static void transfer_completion_work_handler(struct k_work *work)
{
    struct usb_transfer *transfer = CONTAINER_OF(work, struct usb_transfer, completion_work);
    
    LOG_DBG("Transfer completed: EP%d, status=%d, transferred=%d",
            transfer->ep_addr & 0x0F, transfer->status, transfer->transferred);
    
    /* Call completion callback */
    if (transfer->callback) {
        transfer->callback(transfer, transfer->user_data);
    }
    
    /* Find the device (this is a bit hacky, but works for now) */
    struct usb_stack_device *dev = usb_stack_get_device();
    if (dev) {
        /* Remove from queue */
        uint8_t ep_num = transfer->ep_addr & 0x0F;
        sys_slist_find_and_remove(&dev->function_driver.ep_transfer_queues[ep_num], &transfer->node);
        
        /* Free transfer */
        free_transfer(dev, transfer);
    }
}

/**
 * @brief Handle transfer completion (called from hardware layer)
 */
void usb_function_driver_transfer_complete(struct usb_stack_device *dev,
                                          struct usb_transfer *transfer,
                                          usb_transfer_status_t status,
                                          size_t transferred)
{
    if (!dev || !transfer) {
        LOG_ERR("Invalid parameters");
        return;
    }
    
    /* Update transfer status */
    transfer->status = status;
    transfer->transferred = transferred;
    
    /* Schedule completion work */
    k_work_init(&transfer->completion_work, transfer_completion_work_handler);
    k_work_submit_to_queue(&dev->function_driver.transfer_work_queue, &transfer->completion_work);
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
        *active_transfers = dev->function_driver.active_transfers;
    }
    
    if (completed_transfers) {
        *completed_transfers = dev->function_driver.stats.completed_transfers;
    }
    
    if (failed_transfers) {
        *failed_transfers = dev->function_driver.stats.failed_transfers;
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
                                              usb_transfer_callback_t callback,
                                              void *user_data)
{
    if (!dev || !setup) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    LOG_DBG("Setting up control transfer: bmRequestType=0x%02x, bRequest=0x%02x, wValue=0x%04x, wIndex=0x%04x, wLength=%d",
            setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);
    
    /* Allocate transfer for data stage (if needed) */
    if (data_length > 0 && data_buffer) {
        uint8_t ep_addr = (setup->bmRequestType & USB_DIR_IN) ? 0x80 : 0x00;
        return usb_function_driver_submit_transfer(dev, ep_addr, data_buffer, data_length, callback, user_data);
    }
    
    /* Status stage only */
    uint8_t status_ep = (setup->bmRequestType & USB_DIR_IN) ? 0x00 : 0x80;
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
    
    uint8_t ep_num = ep_addr & 0x0F;
    if (ep_num >= USB_STACK_MAX_ENDPOINTS) {
        LOG_ERR("Invalid endpoint number: %d", ep_num);
        return -EINVAL;
    }
    
    LOG_DBG("Stalling endpoint %d", ep_num);
    
    /* Stall in hardware */
    return dwc3_controller_stall_endpoint(&dev->controller, ep_addr);
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
    
    uint8_t ep_num = ep_addr & 0x0F;
    if (ep_num >= USB_STACK_MAX_ENDPOINTS) {
        LOG_ERR("Invalid endpoint number: %d", ep_num);
        return -EINVAL;
    }
    
    LOG_DBG("Clearing stall on endpoint %d", ep_num);
    
    /* Clear stall in hardware */
    return dwc3_controller_clear_stall(&dev->controller, ep_addr);
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
    
    uint8_t ep_num = ep_addr & 0x0F;
    if (ep_num >= USB_STACK_MAX_ENDPOINTS) {
        LOG_ERR("Invalid endpoint number: %d", ep_num);
        return false;
    }
    
    /* Check stall status in hardware */
    return dwc3_controller_is_endpoint_stalled(&dev->controller, ep_addr);
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
    
    uint8_t ep_num = ep_addr & 0x0F;
    if (ep_num >= USB_STACK_MAX_ENDPOINTS) {
        LOG_ERR("Invalid endpoint number: %d", ep_num);
        return -EINVAL;
    }
    
    LOG_DBG("Flushing endpoint %d", ep_num);
    
    /* Cancel all pending transfers for this endpoint */
    sys_snode_t *node, *next;
    SYS_SLIST_FOR_EACH_NODE_SAFE(&dev->function_driver.ep_transfer_queues[ep_num], node, next) {
        struct usb_transfer *transfer = CONTAINER_OF(node, struct usb_transfer, node);
        usb_function_driver_cancel_transfer(dev, transfer);
    }
    
    /* Flush in hardware */
    return dwc3_controller_flush_endpoint(&dev->controller, ep_addr);
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
    
    uint8_t ep_num = ep_addr & 0x0F;
    if (ep_num >= USB_STACK_MAX_ENDPOINTS) {
        LOG_ERR("Invalid endpoint number: %d", ep_num);
        return -EINVAL;
    }
    
    int depth = 0;
    sys_snode_t *node;
    SYS_SLIST_FOR_EACH_NODE(&dev->function_driver.ep_transfer_queues[ep_num], node) {
        depth++;
    }
    
    return depth;
}
