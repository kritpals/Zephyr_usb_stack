/*
 * DWC3 USB Controller Driver for Zephyr USB Stack
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>

#include "../include/usb_stack.h"
#include "../hardware/include/dwc3_regs.h"

LOG_MODULE_REGISTER(dwc3_controller, CONFIG_USB_STACK_LOG_LEVEL);

/* DWC3 Controller Private Data */
struct dwc3_controller_data {
    /* Base address */
    uintptr_t base;
    
    /* Hardware parameters */
    uint32_t hwparams[8];
    
    /* Event buffer */
    struct dwc3_event_buffer *ev_buf;
    
    /* Endpoint data */
    struct dwc3_ep {
        struct usb_stack_endpoint *ep;
        struct dwc3_trb *trb_pool;
        uintptr_t trb_pool_dma;
        uint32_t trb_enqueue;
        uint32_t trb_dequeue;
        uint32_t resource_index;
        bool busy;
    } eps[DWC3_ENDPOINTS_NUM];
    
    /* Device state */
    usb_stack_speed_t speed;
    uint8_t test_mode;
    bool setup_packet_pending;
    
    /* Synchronization */
    struct k_spinlock lock;
    
    /* IRQ */
    unsigned int irq;
    
    /* Connected USB stack device */
    struct usb_stack_device *usb_dev;
};

/* DWC3 Controller Configuration */
struct dwc3_controller_config {
    uintptr_t base;
    unsigned int irq;
    const struct device *clock_dev;
    const struct device *reset_dev;
    uint32_t clock_id;
    uint32_t reset_id;
};

/* Register access helpers */
static inline uint32_t dwc3_readl(struct dwc3_controller_data *dwc, uint32_t offset)
{
    return sys_read32(dwc->base + offset);
}

static inline void dwc3_writel(struct dwc3_controller_data *dwc, uint32_t offset, uint32_t value)
{
    sys_write32(value, dwc->base + offset);
}

/* Hardware parameter reading */
static void dwc3_read_hwparams(struct dwc3_controller_data *dwc)
{
    for (int i = 0; i < 8; i++) {
        dwc->hwparams[i] = dwc3_readl(dwc, DWC3_GHWPARAMS0 + (i * 4));
        LOG_DBG("GHWPARAMS%d: 0x%08x", i, dwc->hwparams[i]);
    }
}

/* Core soft reset */
static int dwc3_core_soft_reset(struct dwc3_controller_data *dwc)
{
    uint32_t reg;
    int timeout = 1000;
    
    /* Clear run/stop bit */
    reg = dwc3_readl(dwc, DWC3_DCTL);
    reg &= ~DWC3_DCTL_RUN_STOP;
    dwc3_writel(dwc, DWC3_DCTL, reg);
    
    /* Issue core soft reset */
    reg = dwc3_readl(dwc, DWC3_GCTL);
    reg |= DWC3_GCTL_CORESOFTRESET;
    dwc3_writel(dwc, DWC3_GCTL, reg);
    
    /* Wait for reset completion */
    do {
        reg = dwc3_readl(dwc, DWC3_GCTL);
        if (!(reg & DWC3_GCTL_CORESOFTRESET)) {
            break;
        }
        k_usleep(1);
    } while (--timeout);
    
    if (!timeout) {
        LOG_ERR("Core soft reset timeout");
        return -ETIMEDOUT;
    }
    
    return 0;
}

/* Core initialization */
static int dwc3_core_init(struct dwc3_controller_data *dwc)
{
    uint32_t reg;
    int ret;
    
    /* Read hardware parameters */
    dwc3_read_hwparams(dwc);
    
    /* Perform core soft reset */
    ret = dwc3_core_soft_reset(dwc);
    if (ret) {
        return ret;
    }
    
    /* Configure global control register */
    reg = dwc3_readl(dwc, DWC3_GCTL);
    
    /* Set port capability to device mode */
    reg &= ~DWC3_GCTL_PRTCAP_MASK;
    reg |= DWC3_GCTL_PRTCAP_DEVICE;
    
    /* Disable clock gating */
    reg |= DWC3_GCTL_DSBLCLKGTNG;
    
    /* Set scale down if needed */
    if (dwc->hwparams[1] & DWC3_GHWPARAMS1_EN_PWROPT_MASK) {
        reg &= ~DWC3_GCTL_SCALEDOWN_MASK;
        reg |= DWC3_GCTL_SCALEDOWN(2);
    }
    
    dwc3_writel(dwc, DWC3_GCTL, reg);
    
    return 0;
}

/* Device initialization */
static int dwc3_device_init(struct dwc3_controller_data *dwc)
{
    uint32_t reg;
    
    /* Configure device */
    reg = dwc3_readl(dwc, DWC3_DCFG);
    
    /* Set device speed to SuperSpeed */
    reg &= ~DWC3_DCFG_SPEED_MASK;
    reg |= DWC3_DCFG_SUPERSPEED;
    
    /* Set number of receive buffers */
    reg &= ~(0x1f << DWC3_DCFG_NUMP_SHIFT);
    reg |= (16 << DWC3_DCFG_NUMP_SHIFT);
    
    dwc3_writel(dwc, DWC3_DCFG, reg);
    
    /* Enable device events */
    reg = DWC3_DEVTEN_DISCONNEVTEN |
          DWC3_DEVTEN_USBRSTEN |
          DWC3_DEVTEN_CONNECTDONEEN |
          DWC3_DEVTEN_ULSTCNGEN |
          DWC3_DEVTEN_WKUPEVTEN |
          DWC3_DEVTEN_SOFEN;
    
    dwc3_writel(dwc, DWC3_DEVTEN, reg);
    
    return 0;
}

/* Event buffer initialization */
static int dwc3_event_buffers_init(struct dwc3_controller_data *dwc)
{
    struct dwc3_event_buffer *evt;
    
    evt = k_malloc(sizeof(*evt));
    if (!evt) {
        return -ENOMEM;
    }
    
    evt->buf = k_aligned_alloc(64, DWC3_EVENT_BUFFERS_SIZE);
    if (!evt->buf) {
        k_free(evt);
        return -ENOMEM;
    }
    
    evt->length = DWC3_EVENT_BUFFERS_SIZE;
    evt->lpos = 0;
    evt->count = 0;
    evt->flags = 0;
    
    dwc->ev_buf = evt;
    
    /* Configure event buffer in hardware */
    dwc3_writel(dwc, DWC3_GEVNTADRLO(0), (uint32_t)(uintptr_t)evt->buf);
    dwc3_writel(dwc, DWC3_GEVNTADRHI(0), 0);
    dwc3_writel(dwc, DWC3_GEVNTSIZ(0), DWC3_EVENT_BUFFERS_SIZE);
    dwc3_writel(dwc, DWC3_GEVNTCOUNT(0), 0);
    
    return 0;
}

/* Endpoint command execution */
static int dwc3_send_ep_cmd(struct dwc3_controller_data *dwc, unsigned int ep,
                           unsigned int cmd, uint32_t param0, uint32_t param1, uint32_t param2)
{
    uint32_t reg;
    int timeout = 1000;
    
    dwc3_writel(dwc, DWC3_DEPCMDPAR0(ep), param0);
    dwc3_writel(dwc, DWC3_DEPCMDPAR1(ep), param1);
    dwc3_writel(dwc, DWC3_DEPCMDPAR2(ep), param2);
    
    reg = cmd | DWC3_DEPCMD_CMDACT;
    dwc3_writel(dwc, DWC3_DEPCMD(ep), reg);
    
    do {
        reg = dwc3_readl(dwc, DWC3_DEPCMD(ep));
        if (!(reg & DWC3_DEPCMD_CMDACT)) {
            return 0;
        }
        k_usleep(1);
    } while (--timeout);
    
    LOG_ERR("Endpoint command timeout for EP%d", ep);
    return -ETIMEDOUT;
}

/* Endpoint configuration */
static int dwc3_configure_endpoint(const struct device *dev, struct usb_stack_endpoint *ep)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint8_t ep_num = USB_STACK_EP_NUM(ep->address);
    uint8_t ep_dir = USB_STACK_EP_DIR(ep->address);
    uint32_t param0, param1, param2;
    int ret;
    
    LOG_DBG("Configuring EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
    
    /* Set endpoint configuration parameters */
    param0 = DWC3_DEPCFG_EP_TYPE(ep->type) |
             DWC3_DEPCFG_MAX_PACKET_SIZE(ep->max_packet_size);
    
    if (ep_num != 0) {
        param0 |= DWC3_DEPCFG_EP_NUMBER(ep_num);
        if (ep_dir) {
            param0 |= DWC3_DEPCFG_FIFO_NUMBER(ep_num);
        }
    }
    
    param1 = DWC3_DEPCFG_XFER_COMPLETE_EN |
             DWC3_DEPCFG_XFER_NOT_READY_EN;
    
    param2 = 0;
    
    ret = dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_SETEPCONFIG,
                          param0, param1, param2);
    if (ret) {
        return ret;
    }
    
    /* Set transfer resource for non-control endpoints */
    if (ep_num != 0) {
        ret = dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_SETTRANSFRESOURCE,
                              1, 0, 0);
        if (ret) {
            return ret;
        }
    }
    
    ep->enabled = true;
    return 0;
}

/* Endpoint deconfiguration */
static int dwc3_deconfigure_endpoint(const struct device *dev, uint8_t ep_addr)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint8_t ep_num = USB_STACK_EP_NUM(ep_addr);
    uint8_t ep_dir = USB_STACK_EP_DIR(ep_addr);
    
    LOG_DBG("Deconfiguring EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
    
    /* End any active transfers */
    dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_ENDTRANSFER, 0, 0, 0);
    
    return 0;
}

/* Transfer submission */
static int dwc3_submit_transfer(const struct device *dev, struct usb_stack_transfer *transfer)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint8_t ep_num = USB_STACK_EP_NUM(transfer->endpoint);
    uint8_t ep_dir = USB_STACK_EP_DIR(transfer->endpoint);
    struct dwc3_ep *dep = &dwc->eps[ep_num * 2 + ep_dir];
    struct dwc3_trb *trb;
    uint32_t param0, param1, param2;
    int ret;
    
    LOG_DBG("Submitting transfer EP%d %s, len=%d", ep_num, ep_dir ? "IN" : "OUT", transfer->length);
    
    if (dep->busy) {
        return -EBUSY;
    }
    
    /* Allocate TRB if not already done */
    if (!dep->trb_pool) {
        dep->trb_pool = k_aligned_alloc(16, sizeof(struct dwc3_trb) * 256);
        if (!dep->trb_pool) {
            return -ENOMEM;
        }
        dep->trb_pool_dma = (uintptr_t)dep->trb_pool;
    }
    
    /* Setup TRB */
    trb = &dep->trb_pool[dep->trb_enqueue];
    trb->bpl = (uint32_t)(uintptr_t)transfer->buffer;
    trb->bph = 0;
    trb->size = transfer->length;
    trb->ctrl = DWC3_TRB_CTRL_HWO | DWC3_TRB_CTRL_LST | DWC3_TRB_CTRL_IOC;
    
    if (ep_num == 0) {
        /* Control endpoint */
        if (transfer->length == 0) {
            trb->ctrl |= DWC3_TRB_CTRL_TRBCTL(DWC3_TRBCTL_CONTROL_STATUS2);
        } else {
            trb->ctrl |= DWC3_TRB_CTRL_TRBCTL(DWC3_TRBCTL_CONTROL_DATA);
        }
    } else {
        /* Non-control endpoint */
        trb->ctrl |= DWC3_TRB_CTRL_TRBCTL(DWC3_TRBCTL_NORMAL);
    }
    
    /* Cache management */
    if (transfer->buffer && transfer->length) {
        if (ep_dir) {
            /* OUT transfer - clean cache */
            sys_cache_data_flush_range(transfer->buffer, transfer->length);
        } else {
            /* IN transfer - invalidate cache */
            sys_cache_data_invd_range(transfer->buffer, transfer->length);
        }
    }
    
    /* Start transfer */
    param0 = (uint32_t)(uintptr_t)trb;
    param1 = 0;
    param2 = 0;
    
    ret = dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_STARTTRANSFER,
                          param0, param1, param2);
    if (ret) {
        return ret;
    }
    
    dep->busy = true;
    transfer->status = USB_STACK_TRANSFER_ACTIVE;
    
    return 0;
}

/* Transfer cancellation */
static int dwc3_cancel_transfer(const struct device *dev, struct usb_stack_transfer *transfer)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint8_t ep_num = USB_STACK_EP_NUM(transfer->endpoint);
    uint8_t ep_dir = USB_STACK_EP_DIR(transfer->endpoint);
    struct dwc3_ep *dep = &dwc->eps[ep_num * 2 + ep_dir];
    
    LOG_DBG("Cancelling transfer EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
    
    if (!dep->busy) {
        return -EINVAL;
    }
    
    /* End transfer */
    dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_ENDTRANSFER,
                    dep->resource_index, 0, 0);
    
    dep->busy = false;
    transfer->status = USB_STACK_TRANSFER_CANCELLED;
    
    return 0;
}

/* Endpoint stall */
static int dwc3_stall_endpoint(const struct device *dev, uint8_t ep_addr)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint8_t ep_num = USB_STACK_EP_NUM(ep_addr);
    uint8_t ep_dir = USB_STACK_EP_DIR(ep_addr);
    
    LOG_DBG("Stalling EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
    
    return dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_SETSTALL, 0, 0, 0);
}

/* Endpoint unstall */
static int dwc3_unstall_endpoint(const struct device *dev, uint8_t ep_addr)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint8_t ep_num = USB_STACK_EP_NUM(ep_addr);
    uint8_t ep_dir = USB_STACK_EP_DIR(ep_addr);
    
    LOG_DBG("Unstalling EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
    
    return dwc3_send_ep_cmd(dwc, ep_num * 2 + ep_dir, DWC3_DEPCMD_CLEARSTALL, 0, 0, 0);
}

/* Set device address */
static int dwc3_set_address(const struct device *dev, uint8_t address)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint32_t reg;
    
    LOG_DBG("Setting device address to %d", address);
    
    reg = dwc3_readl(dwc, DWC3_DCFG);
    reg &= ~DWC3_DCFG_DEVADDR_MASK;
    reg |= DWC3_DCFG_DEVADDR(address);
    dwc3_writel(dwc, DWC3_DCFG, reg);
    
    return 0;
}

/* Get current speed */
static usb_stack_speed_t dwc3_get_speed(const struct device *dev)
{
    struct dwc3_controller_data *dwc = dev->data;
    return dwc->speed;
}

/* Event processing */
static void dwc3_process_device_event(struct dwc3_controller_data *dwc, uint32_t event)
{
    uint8_t event_type = (event >> 8) & 0xf;
    
    switch (event_type) {
    case DWC3_DEVICE_EVENT_DISCONNECT:
        LOG_INF("Device disconnected");
        dwc->speed = USB_STACK_SPEED_UNKNOWN;
        if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
            dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_DISCONNECT, NULL);
        }
        break;
        
    case DWC3_DEVICE_EVENT_RESET:
        LOG_INF("Device reset");
        dwc->speed = USB_STACK_SPEED_UNKNOWN;
        if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
            dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_RESET, NULL);
        }
        break;
        
    case DWC3_DEVICE_EVENT_CONNECT_DONE:
        LOG_INF("Connection done");
        /* Determine speed from DSTS register */
        uint32_t dsts = dwc3_readl(dwc, DWC3_DSTS);
        uint32_t speed = dsts & DWC3_DSTS_CONNECTSPD;
        
        switch (speed) {
        case DWC3_DCFG_SUPERSPEED_PLUS:
            dwc->speed = USB_STACK_SPEED_SUPER_PLUS;
            break;
        case DWC3_DCFG_SUPERSPEED:
            dwc->speed = USB_STACK_SPEED_SUPER;
            break;
        case DWC3_DCFG_HIGHSPEED:
            dwc->speed = USB_STACK_SPEED_HIGH;
            break;
        case DWC3_DCFG_FULLSPEED1:
        case DWC3_DCFG_FULLSPEED2:
            dwc->speed = USB_STACK_SPEED_FULL;
            break;
        case DWC3_DCFG_LOWSPEED:
            dwc->speed = USB_STACK_SPEED_LOW;
            break;
        }
        
        LOG_INF("Connected at %s speed", 
                dwc->speed == USB_STACK_SPEED_SUPER_PLUS ? "SuperSpeed+" :
                dwc->speed == USB_STACK_SPEED_SUPER ? "SuperSpeed" :
                dwc->speed == USB_STACK_SPEED_HIGH ? "High" :
                dwc->speed == USB_STACK_SPEED_FULL ? "Full" : "Low");
        
        if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
            dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_CONNECT, NULL);
        }
        break;
        
    case DWC3_DEVICE_EVENT_WAKEUP:
        LOG_DBG("Wakeup event");
        if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
            dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_RESUME, NULL);
        }
        break;
        
    case DWC3_DEVICE_EVENT_SOF:
        LOG_DBG("SOF event");
        if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
            dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_SOF, NULL);
        }
        break;
        
    default:
        LOG_DBG("Unknown device event: %d", event_type);
        break;
    }
}

static void dwc3_process_endpoint_event(struct dwc3_controller_data *dwc, uint32_t event)
{
    uint8_t ep_num = (event >> 1) & 0xf;
    uint8_t ep_dir = (event & 1);
    uint8_t event_type = (event >> 6) & 0xf;
    struct dwc3_ep *dep = &dwc->eps[ep_num * 2 + ep_dir];
    
    LOG_DBG("EP%d %s event: %d", ep_num, ep_dir ? "IN" : "OUT", event_type);
    
    switch (event_type) {
    case DWC3_DEPEVT_XFERCOMPLETE:
        LOG_DBG("Transfer complete EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
        dep->busy = false;
        /* Notify upper layer */
        if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
            dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_TRANSFER_COMPLETE, 
                                                &dep->ep->address);
        }
        break;
        
    case DWC3_DEPEVT_XFERNOTREADY:
        LOG_DBG("Transfer not ready EP%d %s", ep_num, ep_dir ? "IN" : "OUT");
        if (ep_num == 0) {
            /* Control endpoint setup phase */
            if (dwc->usb_dev && dwc->usb_dev->config->event_callback) {
                dwc->usb_dev->config->event_callback(dwc->usb_dev, USB_STACK_EVENT_SETUP, NULL);
            }
        }
        break;
        
    default:
        LOG_DBG("Unknown endpoint event: %d", event_type);
        break;
    }
}

/* Event buffer processing */
static void dwc3_process_events(struct dwc3_controller_data *dwc)
{
    struct dwc3_event_buffer *evt = dwc->ev_buf;
    uint32_t count;
    uint32_t *event_ptr;
    
    count = dwc3_readl(dwc, DWC3_GEVNTCOUNT(0));
    count &= 0xffff;
    
    if (!count) {
        return;
    }
    
    event_ptr = (uint32_t *)((uint8_t *)evt->buf + evt->lpos);
    
    while (count > 0) {
        uint32_t event = *event_ptr;
        
        if (event & 1) {
            /* Device event */
            dwc3_process_device_event(dwc, event);
        } else {
            /* Endpoint event */
            dwc3_process_endpoint_event(dwc, event);
        }
        
        event_ptr++;
        evt->lpos += 4;
        count -= 4;
        
        if (evt->lpos >= evt->length) {
            evt->lpos = 0;
            event_ptr = (uint32_t *)evt->buf;
        }
    }
    
    /* Update event count register */
    dwc3_writel(dwc, DWC3_GEVNTCOUNT(0), count);
}

/* Interrupt handler */
static void dwc3_isr(const struct device *dev)
{
    struct dwc3_controller_data *dwc = dev->data;
    k_spinlock_key_t key;
    
    key = k_spin_lock(&dwc->lock);
    dwc3_process_events(dwc);
    k_spin_unlock(&dwc->lock, key);
}

/* Controller API implementation */
static int dwc3_controller_init(const struct device *dev)
{
    struct dwc3_controller_data *dwc = dev->data;
    const struct dwc3_controller_config *config = dev->config;
    int ret;
    
    LOG_INF("Initializing DWC3 controller");
    
    dwc->base = config->base;
    
    /* Initialize core */
    ret = dwc3_core_init(dwc);
    if (ret) {
        LOG_ERR("Core initialization failed: %d", ret);
        return ret;
    }
    
    /* Initialize device mode */
    ret = dwc3_device_init(dwc);
    if (ret) {
        LOG_ERR("Device initialization failed: %d", ret);
        return ret;
    }
    
    /* Initialize event buffers */
    ret = dwc3_event_buffers_init(dwc);
    if (ret) {
        LOG_ERR("Event buffer initialization failed: %d", ret);
        return ret;
    }
    
    /* Configure interrupt - store IRQ number for later use */
    dwc->irq = config->irq;
    
    /* Enable interrupt */
    irq_enable(config->irq);
    
    LOG_INF("DWC3 controller initialized successfully");
    return 0;
}

static int dwc3_controller_enable(const struct device *dev)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint32_t reg;
    
    LOG_INF("Enabling DWC3 controller");
    
    /* Start the controller */
    reg = dwc3_readl(dwc, DWC3_DCTL);
    reg |= DWC3_DCTL_RUN_STOP;
    dwc3_writel(dwc, DWC3_DCTL, reg);
    
    return 0;
}

static int dwc3_controller_disable(const struct device *dev)
{
    struct dwc3_controller_data *dwc = dev->data;
    uint32_t reg;
    
    LOG_INF("Disabling DWC3 controller");
    
    /* Stop the controller */
    reg = dwc3_readl(dwc, DWC3_DCTL);
    reg &= ~DWC3_DCTL_RUN_STOP;
    dwc3_writel(dwc, DWC3_DCTL, reg);
    
    return 0;
}

static int dwc3_controller_reset(const struct device *dev)
{
    struct dwc3_controller_data *dwc = dev->data;
    
    LOG_INF("Resetting DWC3 controller");
    
    return dwc3_core_soft_reset(dwc);
}

/* Controller API structure */
static const struct usb_stack_controller_api dwc3_controller_api = {
    .init = dwc3_controller_init,
    .deinit = NULL,
    .enable = dwc3_controller_enable,
    .disable = dwc3_controller_disable,
    .reset = dwc3_controller_reset,
    .set_address = dwc3_set_address,
    .configure_endpoint = dwc3_configure_endpoint,
    .deconfigure_endpoint = dwc3_deconfigure_endpoint,
    .stall_endpoint = dwc3_stall_endpoint,
    .unstall_endpoint = dwc3_unstall_endpoint,
    .submit_transfer = dwc3_submit_transfer,
    .cancel_transfer = dwc3_cancel_transfer,
    .get_speed = dwc3_get_speed,
};

/* Device instantiation macro */
#define DWC3_CONTROLLER_INIT(n)                                                \
    static struct dwc3_controller_data dwc3_data_##n = {                      \
        .speed = USB_STACK_SPEED_UNKNOWN,                                     \
    };                                                                         \
                                                                               \
    static const struct dwc3_controller_config dwc3_config_##n = {            \
        .base = DT_INST_REG_ADDR(n),                                         \
        .irq = DT_INST_IRQN(n),                                              \
    };                                                                         \
                                                                               \
    DEVICE_DT_INST_DEFINE(n, dwc3_controller_init, NULL,                     \
                          &dwc3_data_##n, &dwc3_config_##n,                  \
                          POST_KERNEL, CONFIG_USB_STACK_INIT_PRIORITY,        \
                          &dwc3_controller_api);

DT_INST_FOREACH_STATUS_OKAY(DWC3_CONTROLLER_INIT)
