/*
 * USB Type-C Manager for Zephyr USB Stack
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

#include "../include/usb_stack.h"

LOG_MODULE_REGISTER(typec_manager, CONFIG_USB_STACK_LOG_LEVEL);

/* Type-C Port Controller Registers */
#define TCPC_VENDOR_ID          0x00
#define TCPC_PRODUCT_ID         0x02
#define TCPC_DEVICE_ID          0x04
#define TCPC_USBTYPEC_REV       0x06
#define TCPC_USBPD_REV_VER      0x08
#define TCPC_PD_INTERFACE_REV   0x0A
#define TCPC_ALERT              0x10
#define TCPC_ALERT_MASK         0x12
#define TCPC_POWER_STATUS_MASK  0x14
#define TCPC_FAULT_STATUS_MASK  0x15
#define TCPC_CONFIG_STD_OUTPUT  0x18
#define TCPC_TCPC_CTRL          0x19
#define TCPC_ROLE_CTRL          0x1A
#define TCPC_FAULT_CTRL         0x1B
#define TCPC_POWER_CTRL         0x1C
#define TCPC_CC_STATUS          0x1D
#define TCPC_POWER_STATUS       0x1E
#define TCPC_FAULT_STATUS       0x1F
#define TCPC_COMMAND            0x23
#define TCPC_DEV_CAP_1          0x24
#define TCPC_DEV_CAP_2          0x26
#define TCPC_STD_INPUT_CAP      0x28
#define TCPC_STD_OUTPUT_CAP     0x29
#define TCPC_MSG_HDR_INFO       0x2E
#define TCPC_RX_DETECT          0x2F
#define TCPC_RX_BYTE_CNT        0x30
#define TCPC_RX_BUF_FRAME_TYPE  0x31
#define TCPC_RX_HDR             0x32
#define TCPC_RX_DATA            0x34
#define TCPC_TRANSMIT           0x50
#define TCPC_TX_BYTE_CNT        0x51
#define TCPC_TX_HDR             0x52
#define TCPC_TX_DATA            0x54

/* Type-C Manager Private Data */
struct typec_manager_data {
    /* Base address for TCPC */
    uintptr_t tcpc_base;
    
    /* GPIO pins */
    struct gpio_dt_spec cc1_pin;
    struct gpio_dt_spec cc2_pin;
    struct gpio_dt_spec vbus_pin;
    struct gpio_dt_spec orientation_pin;
    
    /* Current state */
    usb_stack_typec_state_t state;
    usb_stack_typec_orientation_t orientation;
    
    /* Connection detection */
    bool cc1_connected;
    bool cc2_connected;
    bool vbus_present;
    
    /* Power delivery */
    bool pd_capable;
    uint16_t current_limit;
    uint16_t voltage;
    
    /* State machine */
    struct k_work_delayable state_work;
    struct k_timer debounce_timer;
    
    /* Callbacks */
    void (*connection_callback)(usb_stack_typec_state_t state, void *user_data);
    void *callback_user_data;
    
    /* Synchronization */
    struct k_mutex lock;
    
    /* Statistics */
    uint32_t connection_count;
    uint32_t disconnection_count;
    uint32_t orientation_changes;
};

/* Type-C Manager Configuration */
struct typec_manager_config {
    uintptr_t tcpc_base;
    struct gpio_dt_spec cc1_pin;
    struct gpio_dt_spec cc2_pin;
    struct gpio_dt_spec vbus_pin;
    struct gpio_dt_spec orientation_pin;
    bool pd_support;
};

/* Register access helpers */
static inline uint16_t tcpc_read16(struct typec_manager_data *typec, uint8_t reg)
{
    return sys_read16(typec->tcpc_base + reg);
}

static inline void tcpc_write16(struct typec_manager_data *typec, uint8_t reg, uint16_t val)
{
    sys_write16(val, typec->tcpc_base + reg);
}

static inline uint8_t tcpc_read8(struct typec_manager_data *typec, uint8_t reg)
{
    return sys_read8(typec->tcpc_base + reg);
}

static inline void tcpc_write8(struct typec_manager_data *typec, uint8_t reg, uint8_t val)
{
    sys_write8(val, typec->tcpc_base + reg);
}

/* CC line detection */
static bool typec_detect_cc_connection(struct typec_manager_data *typec)
{
    uint8_t cc_status;
    bool cc1_connected = false;
    bool cc2_connected = false;
    
    /* Read CC status from TCPC */
    cc_status = tcpc_read8(typec, TCPC_CC_STATUS);
    
    /* Check CC1 connection */
    uint8_t cc1_state = (cc_status >> 0) & 0x3;
    if (cc1_state == 0x1 || cc1_state == 0x2) {  /* Rd or Ra */
        cc1_connected = true;
    }
    
    /* Check CC2 connection */
    uint8_t cc2_state = (cc_status >> 2) & 0x3;
    if (cc2_state == 0x1 || cc2_state == 0x2) {  /* Rd or Ra */
        cc2_connected = true;
    }
    
    /* Update connection state */
    bool connection_changed = (typec->cc1_connected != cc1_connected) ||
                             (typec->cc2_connected != cc2_connected);
    
    typec->cc1_connected = cc1_connected;
    typec->cc2_connected = cc2_connected;
    
    /* Determine orientation */
    usb_stack_typec_orientation_t new_orientation = USB_STACK_TYPEC_ORIENTATION_NONE;
    
    if (cc1_connected && !cc2_connected) {
        new_orientation = USB_STACK_TYPEC_ORIENTATION_CC1;
    } else if (!cc1_connected && cc2_connected) {
        new_orientation = USB_STACK_TYPEC_ORIENTATION_CC2;
    }
    
    if (typec->orientation != new_orientation) {
        typec->orientation = new_orientation;
        typec->orientation_changes++;
        
        /* Update orientation GPIO if available */
        if (typec->orientation_pin.port) {
            gpio_pin_set_dt(&typec->orientation_pin, 
                           (new_orientation == USB_STACK_TYPEC_ORIENTATION_CC2) ? 1 : 0);
        }
        
        LOG_DBG("Type-C orientation changed to %s", 
                new_orientation == USB_STACK_TYPEC_ORIENTATION_CC1 ? "CC1" :
                new_orientation == USB_STACK_TYPEC_ORIENTATION_CC2 ? "CC2" : "None");
    }
    
    return connection_changed;
}

/* VBUS detection */
static bool typec_detect_vbus(struct typec_manager_data *typec)
{
    bool vbus_present = false;
    
    /* Check VBUS from TCPC power status */
    uint8_t power_status = tcpc_read8(typec, TCPC_POWER_STATUS);
    if (power_status & BIT(2)) {  /* VBUS present */
        vbus_present = true;
    }
    
    /* Also check GPIO if available */
    if (typec->vbus_pin.port) {
        int gpio_state = gpio_pin_get_dt(&typec->vbus_pin);
        vbus_present = vbus_present || (gpio_state > 0);
    }
    
    bool vbus_changed = (typec->vbus_present != vbus_present);
    typec->vbus_present = vbus_present;
    
    return vbus_changed;
}

/* State machine update */
static void typec_update_state(struct typec_manager_data *typec)
{
    usb_stack_typec_state_t new_state = USB_STACK_TYPEC_UNATTACHED;
    
    /* Determine new state based on CC and VBUS status */
    if (typec->cc1_connected || typec->cc2_connected) {
        if (typec->vbus_present) {
            new_state = USB_STACK_TYPEC_ATTACHED_SNK;
        } else {
            new_state = USB_STACK_TYPEC_ATTACHED_SRC;
        }
    }
    
    /* Update state if changed */
    if (typec->state != new_state) {
        usb_stack_typec_state_t old_state = typec->state;
        typec->state = new_state;
        
        LOG_INF("Type-C state changed: %d -> %d", old_state, new_state);
        
        /* Update statistics */
        if (new_state != USB_STACK_TYPEC_UNATTACHED) {
            typec->connection_count++;
        } else if (old_state != USB_STACK_TYPEC_UNATTACHED) {
            typec->disconnection_count++;
        }
        
        /* Notify callback */
        if (typec->connection_callback) {
            typec->connection_callback(new_state, typec->callback_user_data);
        }
    }
}

/* Debounce timer callback */
static void typec_debounce_timer_handler(struct k_timer *timer)
{
    struct typec_manager_data *typec = CONTAINER_OF(timer, struct typec_manager_data, debounce_timer);
    
    /* Schedule state machine work */
    k_work_schedule(&typec->state_work, K_NO_WAIT);
}

/* State machine work handler */
static void typec_state_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct typec_manager_data *typec = CONTAINER_OF(dwork, struct typec_manager_data, state_work);
    
    k_mutex_lock(&typec->lock, K_FOREVER);
    
    /* Detect CC and VBUS changes */
    bool cc_changed = typec_detect_cc_connection(typec);
    bool vbus_changed = typec_detect_vbus(typec);
    
    if (cc_changed || vbus_changed) {
        /* Update state machine */
        typec_update_state(typec);
    }
    
    k_mutex_unlock(&typec->lock);
    
    /* Schedule next check */
    k_work_schedule(&typec->state_work, K_MSEC(100));
}

/* TCPC interrupt handler */
static void typec_tcpc_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct typec_manager_data *typec = CONTAINER_OF(cb, struct typec_manager_data, tcpc_callback);
    
    /* Start debounce timer */
    k_timer_start(&typec->debounce_timer, K_MSEC(10), K_NO_WAIT);
}

/* Type-C API implementation */
static int typec_manager_init(const struct device *dev)
{
    struct typec_manager_data *typec = dev->data;
    const struct typec_manager_config *config = dev->config;
    int ret;
    
    LOG_INF("Initializing Type-C Manager");
    
    /* Initialize base address */
    typec->tcpc_base = config->tcpc_base;
    
    /* Initialize GPIO pins */
    if (config->cc1_pin.port) {
        if (!gpio_is_ready_dt(&config->cc1_pin)) {
            LOG_ERR("CC1 GPIO not ready");
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&config->cc1_pin, GPIO_INPUT);
        if (ret) {
            LOG_ERR("Failed to configure CC1 GPIO: %d", ret);
            return ret;
        }
        typec->cc1_pin = config->cc1_pin;
    }
    
    if (config->cc2_pin.port) {
        if (!gpio_is_ready_dt(&config->cc2_pin)) {
            LOG_ERR("CC2 GPIO not ready");
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&config->cc2_pin, GPIO_INPUT);
        if (ret) {
            LOG_ERR("Failed to configure CC2 GPIO: %d", ret);
            return ret;
        }
        typec->cc2_pin = config->cc2_pin;
    }
    
    if (config->vbus_pin.port) {
        if (!gpio_is_ready_dt(&config->vbus_pin)) {
            LOG_ERR("VBUS GPIO not ready");
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&config->vbus_pin, GPIO_INPUT);
        if (ret) {
            LOG_ERR("Failed to configure VBUS GPIO: %d", ret);
            return ret;
        }
        typec->vbus_pin = config->vbus_pin;
    }
    
    if (config->orientation_pin.port) {
        if (!gpio_is_ready_dt(&config->orientation_pin)) {
            LOG_ERR("Orientation GPIO not ready");
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&config->orientation_pin, GPIO_OUTPUT_INACTIVE);
        if (ret) {
            LOG_ERR("Failed to configure orientation GPIO: %d", ret);
            return ret;
        }
        typec->orientation_pin = config->orientation_pin;
    }
    
    /* Initialize TCPC if available */
    if (typec->tcpc_base) {
        /* Reset TCPC */
        tcpc_write8(typec, TCPC_COMMAND, 0x01);  /* I2C_IDLE */
        k_msleep(1);
        
        /* Configure TCPC for DRP mode */
        tcpc_write8(typec, TCPC_ROLE_CTRL, 0x4A);  /* DRP, Rp/Rd */
        
        /* Enable alerts */
        tcpc_write16(typec, TCPC_ALERT_MASK, 0x7FFF);
        
        /* Configure power control */
        tcpc_write8(typec, TCPC_POWER_CTRL, 0x60);  /* Enable VBUS monitoring */
    }
    
    /* Initialize state */
    typec->state = USB_STACK_TYPEC_UNATTACHED;
    typec->orientation = USB_STACK_TYPEC_ORIENTATION_NONE;
    typec->pd_capable = config->pd_support;
    
    /* Initialize work and timer */
    k_work_init_delayable(&typec->state_work, typec_state_work_handler);
    k_timer_init(&typec->debounce_timer, typec_debounce_timer_handler, NULL);
    
    /* Initialize mutex */
    k_mutex_init(&typec->lock);
    
    /* Start state machine */
    k_work_schedule(&typec->state_work, K_MSEC(100));
    
    LOG_INF("Type-C Manager initialized successfully");
    return 0;
}

static int typec_manager_deinit(const struct device *dev)
{
    struct typec_manager_data *typec = dev->data;
    
    LOG_INF("Deinitializing Type-C Manager");
    
    /* Cancel work and timer */
    k_work_cancel_delayable(&typec->state_work);
    k_timer_stop(&typec->debounce_timer);
    
    /* Reset state */
    typec->state = USB_STACK_TYPEC_UNATTACHED;
    typec->orientation = USB_STACK_TYPEC_ORIENTATION_NONE;
    
    return 0;
}

static int typec_manager_enable(const struct device *dev)
{
    struct typec_manager_data *typec = dev->data;
    
    LOG_DBG("Enabling Type-C Manager");
    
    /* Start state machine if not already running */
    k_work_schedule(&typec->state_work, K_NO_WAIT);
    
    return 0;
}

static int typec_manager_disable(const struct device *dev)
{
    struct typec_manager_data *typec = dev->data;
    
    LOG_DBG("Disabling Type-C Manager");
    
    /* Stop state machine */
    k_work_cancel_delayable(&typec->state_work);
    k_timer_stop(&typec->debounce_timer);
    
    return 0;
}

static usb_stack_typec_state_t typec_manager_get_state(const struct device *dev)
{
    struct typec_manager_data *typec = dev->data;
    return typec->state;
}

static usb_stack_typec_orientation_t typec_manager_get_orientation(const struct device *dev)
{
    struct typec_manager_data *typec = dev->data;
    return typec->orientation;
}

static int typec_manager_set_role(const struct device *dev, int role)
{
    struct typec_manager_data *typec = dev->data;
    
    LOG_DBG("Setting Type-C role to %d", role);
    
    /* Configure TCPC role if available */
    if (typec->tcpc_base) {
        uint8_t role_ctrl = tcpc_read8(typec, TCPC_ROLE_CTRL);
        
        switch (role) {
        case 0:  /* Source */
            role_ctrl = (role_ctrl & 0xF0) | 0x05;  /* Rp */
            break;
        case 1:  /* Sink */
            role_ctrl = (role_ctrl & 0xF0) | 0x0A;  /* Rd */
            break;
        case 2:  /* DRP */
            role_ctrl = (role_ctrl & 0xF0) | 0x4A;  /* DRP */
            break;
        default:
            return -EINVAL;
        }
        
        tcpc_write8(typec, TCPC_ROLE_CTRL, role_ctrl);
    }
    
    return 0;
}

/* Type-C API structure */
static const struct usb_stack_typec_api typec_manager_api = {
    .init = typec_manager_init,
    .deinit = typec_manager_deinit,
    .enable = typec_manager_enable,
    .disable = typec_manager_disable,
    .get_state = typec_manager_get_state,
    .get_orientation = typec_manager_get_orientation,
    .set_role = typec_manager_set_role,
};

/* Public API for registering callbacks */
int typec_manager_register_callback(const struct device *dev,
                                   void (*callback)(usb_stack_typec_state_t state, void *user_data),
                                   void *user_data)
{
    struct typec_manager_data *typec = dev->data;
    
    k_mutex_lock(&typec->lock, K_FOREVER);
    typec->connection_callback = callback;
    typec->callback_user_data = user_data;
    k_mutex_unlock(&typec->lock);
    
    return 0;
}

/* Device instantiation macro */
#define TYPEC_MANAGER_INIT(n)                                                 \
    static struct typec_manager_data typec_data_##n = {                      \
        .state = USB_STACK_TYPEC_UNATTACHED,                                 \
        .orientation = USB_STACK_TYPEC_ORIENTATION_NONE,                     \
    };                                                                        \
                                                                              \
    static const struct typec_manager_config typec_config_##n = {            \
        .tcpc_base = DT_INST_REG_ADDR(n),                                   \
        .cc1_pin = GPIO_DT_SPEC_INST_GET_OR(n, cc1_gpios, {0}),            \
        .cc2_pin = GPIO_DT_SPEC_INST_GET_OR(n, cc2_gpios, {0}),            \
        .vbus_pin = GPIO_DT_SPEC_INST_GET_OR(n, vbus_gpios, {0}),          \
        .orientation_pin = GPIO_DT_SPEC_INST_GET_OR(n, orientation_gpios, {0}), \
        .pd_support = DT_INST_PROP(n, pd_support),                          \
    };                                                                        \
                                                                              \
    DEVICE_DT_INST_DEFINE(n, typec_manager_init, NULL,                      \
                          &typec_data_##n, &typec_config_##n,               \
                          POST_KERNEL, CONFIG_USB_STACK_INIT_PRIORITY,       \
                          &typec_manager_api);

DT_INST_FOREACH_STATUS_OKAY(TYPEC_MANAGER_INIT)
