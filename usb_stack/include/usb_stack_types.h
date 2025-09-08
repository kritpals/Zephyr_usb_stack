/*
 * USB Stack Types and Definitions
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USB_STACK_TYPES_H
#define USB_STACK_TYPES_H

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>
#include "usb_stack_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct usb_stack_device;
struct usb_stack_endpoint;
struct usb_stack_transfer;

/* USB Stack Error Codes */
typedef enum {
    USB_STACK_SUCCESS = 0,
    USB_STACK_ERROR_INVALID_PARAM = -1,
    USB_STACK_ERROR_NO_MEMORY = -2,
    USB_STACK_ERROR_TIMEOUT = -3,
    USB_STACK_ERROR_NOT_SUPPORTED = -4,
    USB_STACK_ERROR_BUSY = -5,
    USB_STACK_ERROR_STALLED = -6,
    USB_STACK_ERROR_DISCONNECTED = -7,
    USB_STACK_ERROR_CONTROLLER = -8,
    USB_STACK_ERROR_PHY = -9,
    USB_STACK_ERROR_TYPEC = -10
} usb_stack_error_t;

/* USB Device States */
typedef enum {
    USB_STACK_STATE_DETACHED = 0,
    USB_STACK_STATE_ATTACHED,
    USB_STACK_STATE_POWERED,
    USB_STACK_STATE_DEFAULT,
    USB_STACK_STATE_ADDRESS,
    USB_STACK_STATE_CONFIGURED,
    USB_STACK_STATE_SUSPENDED
} usb_stack_device_state_t;

/* USB Speed Enumeration */
typedef enum {
    USB_STACK_SPEED_UNKNOWN = 0,
    USB_STACK_SPEED_LOW,      /* 1.5 Mbps */
    USB_STACK_SPEED_FULL,     /* 12 Mbps */
    USB_STACK_SPEED_HIGH,     /* 480 Mbps */
    USB_STACK_SPEED_SUPER,    /* 5 Gbps */
    USB_STACK_SPEED_SUPER_PLUS /* 10 Gbps */
} usb_stack_speed_t;

/* Endpoint Types */
typedef enum {
    USB_STACK_EP_TYPE_CONTROL = 0,
    USB_STACK_EP_TYPE_ISOCHRONOUS,
    USB_STACK_EP_TYPE_BULK,
    USB_STACK_EP_TYPE_INTERRUPT
} usb_stack_ep_type_t;

/* Endpoint Direction */
typedef enum {
    USB_STACK_EP_DIR_OUT = 0,
    USB_STACK_EP_DIR_IN = 1
} usb_stack_ep_dir_t;

/* Transfer Status */
typedef enum {
    USB_STACK_TRANSFER_IDLE = 0,
    USB_STACK_TRANSFER_PENDING,
    USB_STACK_TRANSFER_ACTIVE,
    USB_STACK_TRANSFER_COMPLETE,
    USB_STACK_TRANSFER_ERROR,
    USB_STACK_TRANSFER_CANCELLED,
    USB_STACK_TRANSFER_STALLED
} usb_stack_transfer_status_t;

/* USB Events */
typedef enum {
    USB_STACK_EVENT_RESET = 0,
    USB_STACK_EVENT_SUSPEND,
    USB_STACK_EVENT_RESUME,
    USB_STACK_EVENT_CONNECT,
    USB_STACK_EVENT_DISCONNECT,
    USB_STACK_EVENT_SETUP,
    USB_STACK_EVENT_TRANSFER_COMPLETE,
    USB_STACK_EVENT_SOF,
    USB_STACK_EVENT_ERROR
} usb_stack_event_type_t;

/* Type-C Connection States */
typedef enum {
    USB_STACK_TYPEC_UNATTACHED = 0,
    USB_STACK_TYPEC_ATTACHED_SNK,
    USB_STACK_TYPEC_ATTACHED_SRC,
    USB_STACK_TYPEC_POWERED_CABLE,
    USB_STACK_TYPEC_AUDIO_ACCESSORY,
    USB_STACK_TYPEC_DEBUG_ACCESSORY
} usb_stack_typec_state_t;

/* Type-C Orientation */
typedef enum {
    USB_STACK_TYPEC_ORIENTATION_NONE = 0,
    USB_STACK_TYPEC_ORIENTATION_CC1,
    USB_STACK_TYPEC_ORIENTATION_CC2
} usb_stack_typec_orientation_t;

/* PHY Types */
typedef enum {
    USB_STACK_PHY_TYPE_UNKNOWN = 0,
    USB_STACK_PHY_TYPE_QUSB2,
    USB_STACK_PHY_TYPE_QMP_USB3,
    USB_STACK_PHY_TYPE_QMP_USB31
} usb_stack_phy_type_t;

/* Callback function types */
typedef void (*usb_stack_event_callback_t)(struct usb_stack_device *dev, 
                                          usb_stack_event_type_t event, 
                                          void *data);

typedef void (*usb_stack_transfer_callback_t)(struct usb_stack_transfer *transfer);

typedef int (*usb_stack_setup_callback_t)(struct usb_stack_device *dev,
                                         struct usb_setup_packet *setup);

/* USB Stack Transfer Request */
struct usb_stack_transfer {
    /* Transfer parameters */
    uint8_t endpoint;
    uint8_t *buffer;
    uint32_t length;
    uint32_t actual_length;
    
    /* Transfer control */
    usb_stack_transfer_status_t status;
    usb_stack_transfer_callback_t callback;
    void *user_data;
    
    /* Internal fields */
    struct k_sem completion_sem;
    sys_dnode_t node;
    uint32_t flags;
    k_timeout_t timeout;
};

/* USB Stack Endpoint Configuration */
struct usb_stack_endpoint {
    uint8_t address;                    /* Endpoint address (includes direction) */
    usb_stack_ep_type_t type;          /* Endpoint type */
    uint16_t max_packet_size;          /* Maximum packet size */
    uint8_t interval;                  /* Polling interval for interrupt/iso */
    
    /* Endpoint state */
    bool enabled;
    bool stalled;
    
    /* Transfer queue */
    sys_dlist_t transfer_queue;
    struct k_mutex queue_mutex;
    
    /* Statistics */
    uint32_t transfer_count;
    uint32_t error_count;
};

/* USB Stack Device Configuration */
struct usb_stack_device_config {
    /* Device descriptors */
    struct usb_device_descriptor *device_desc;
    struct usb_cfg_descriptor *config_desc;
    struct usb_string_descriptor **string_descs;
    uint8_t num_strings;
    
    /* Callbacks */
    usb_stack_event_callback_t event_callback;
    usb_stack_setup_callback_t setup_callback;
    
    /* Configuration */
    bool self_powered;
    bool remote_wakeup;
    uint16_t max_power;  /* in 2mA units */
};

/* USB Stack Device Instance */
struct usb_stack_device {
    /* Device configuration */
    const struct usb_stack_device_config *config;
    
    /* Device state */
    usb_stack_device_state_t state;
    usb_stack_speed_t speed;
    uint8_t address;
    uint8_t configuration;
    
    /* Endpoints */
    struct usb_stack_endpoint endpoints[USB_STACK_MAX_ENDPOINTS * 2]; /* IN + OUT */
    
    /* Control transfer state */
    struct usb_setup_packet setup_packet;
    uint8_t *control_buffer;
    uint32_t control_length;
    
    /* Event handling */
    struct k_work event_work;
    struct k_msgq event_queue;
    uint8_t event_buffer[16 * sizeof(usb_stack_event_type_t)];
    
    /* Synchronization */
    struct k_mutex device_mutex;
    struct k_sem init_sem;
    
    /* Hardware interfaces */
    const struct device *controller_dev;
    const struct device *phy_dev;
    const struct device *typec_dev;
    
    /* Statistics and debugging */
    uint32_t reset_count;
    uint32_t suspend_count;
    uint32_t resume_count;
    uint32_t setup_count;
    uint32_t transfer_count;
    uint32_t error_count;
};

/* Hardware Abstraction Layer Interfaces */

/* Controller Interface */
struct usb_stack_controller_api {
    int (*init)(const struct device *dev);
    int (*deinit)(const struct device *dev);
    int (*enable)(const struct device *dev);
    int (*disable)(const struct device *dev);
    int (*reset)(const struct device *dev);
    int (*set_address)(const struct device *dev, uint8_t address);
    int (*configure_endpoint)(const struct device *dev, struct usb_stack_endpoint *ep);
    int (*deconfigure_endpoint)(const struct device *dev, uint8_t ep_addr);
    int (*stall_endpoint)(const struct device *dev, uint8_t ep_addr);
    int (*unstall_endpoint)(const struct device *dev, uint8_t ep_addr);
    int (*submit_transfer)(const struct device *dev, struct usb_stack_transfer *transfer);
    int (*cancel_transfer)(const struct device *dev, struct usb_stack_transfer *transfer);
    usb_stack_speed_t (*get_speed)(const struct device *dev);
};

/* PHY Interface */
struct usb_stack_phy_api {
    int (*init)(const struct device *dev);
    int (*deinit)(const struct device *dev);
    int (*power_on)(const struct device *dev);
    int (*power_off)(const struct device *dev);
    int (*set_mode)(const struct device *dev, int mode);
    int (*calibrate)(const struct device *dev);
    usb_stack_phy_type_t (*get_type)(const struct device *dev);
};

/* Type-C Interface */
struct usb_stack_typec_api {
    int (*init)(const struct device *dev);
    int (*deinit)(const struct device *dev);
    int (*enable)(const struct device *dev);
    int (*disable)(const struct device *dev);
    usb_stack_typec_state_t (*get_state)(const struct device *dev);
    usb_stack_typec_orientation_t (*get_orientation)(const struct device *dev);
    int (*set_role)(const struct device *dev, int role);
};

/* Utility Macros */
#define USB_STACK_EP_ADDR(num, dir) ((num) | ((dir) ? 0x80 : 0x00))
#define USB_STACK_EP_NUM(addr) ((addr) & 0x0F)
#define USB_STACK_EP_DIR(addr) (((addr) & 0x80) ? USB_STACK_EP_DIR_IN : USB_STACK_EP_DIR_OUT)
#define USB_STACK_EP_INDEX(addr) (USB_STACK_EP_NUM(addr) * 2 + USB_STACK_EP_DIR(addr))

#define USB_STACK_TRANSFER_FLAG_ZERO_PACKET BIT(0)
#define USB_STACK_TRANSFER_FLAG_SHORT_OK    BIT(1)
#define USB_STACK_TRANSFER_FLAG_DMA         BIT(2)

#ifdef __cplusplus
}
#endif

#endif /* USB_STACK_TYPES_H */
