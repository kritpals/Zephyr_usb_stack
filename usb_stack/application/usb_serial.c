/*
 * USB Serial Communication Application Layer
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "../include/usb_stack.h"

LOG_MODULE_REGISTER(usb_serial, CONFIG_USB_STACK_LOG_LEVEL);

/* USB CDC Class Codes */
#define USB_CDC_CLASS                   0x02
#define USB_CDC_SUBCLASS_ACM            0x02
#define USB_CDC_PROTOCOL_AT             0x01

/* CDC Functional Descriptors */
#define USB_CDC_FUNC_DESC_HEADER        0x00
#define USB_CDC_FUNC_DESC_CALL_MGMT     0x01
#define USB_CDC_FUNC_DESC_ACM           0x02
#define USB_CDC_FUNC_DESC_UNION         0x06

/* CDC Class Requests */
#define USB_CDC_SET_LINE_CODING         0x20
#define USB_CDC_GET_LINE_CODING         0x21
#define USB_CDC_SET_CONTROL_LINE_STATE  0x22
#define USB_CDC_SEND_BREAK              0x23

/* CDC Notifications */
#define USB_CDC_NOTIFY_NETWORK_CONNECTION   0x00
#define USB_CDC_NOTIFY_RESPONSE_AVAILABLE   0x01
#define USB_CDC_NOTIFY_AUX_JACK_HOOK_STATE  0x08
#define USB_CDC_NOTIFY_RING_DETECT          0x09
#define USB_CDC_NOTIFY_SERIAL_STATE         0x20

/* Line Coding Structure */
struct cdc_line_coding {
    uint32_t dwDTERate;     /* Data terminal rate in bits per second */
    uint8_t bCharFormat;    /* Stop bits: 0=1, 1=1.5, 2=2 */
    uint8_t bParityType;    /* Parity: 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
    uint8_t bDataBits;      /* Data bits: 5, 6, 7, 8, 16 */
} __packed;

/* Serial State Notification */
struct cdc_serial_state {
    uint8_t bmRequestType;
    uint8_t bNotification;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
    uint16_t bmUartState;
} __packed;

/* USB Serial Device Data */
struct usb_serial_data {
    /* USB stack device */
    struct usb_stack_device usb_dev;
    
    /* Line coding */
    struct cdc_line_coding line_coding;
    
    /* Control line state */
    uint16_t control_line_state;
    
    /* Serial state */
    uint16_t serial_state;
    
    /* Data endpoints */
    uint8_t data_in_ep;
    uint8_t data_out_ep;
    uint8_t notify_ep;
    
    /* Transfer buffers */
    uint8_t *rx_buffer;
    uint8_t *tx_buffer;
    uint8_t *notify_buffer;
    
    /* Buffer management */
    struct k_fifo rx_fifo;
    struct k_fifo tx_fifo;
    struct k_mutex tx_mutex;
    
    /* Transfer management */
    struct usb_stack_transfer rx_transfer;
    struct usb_stack_transfer tx_transfer;
    struct usb_stack_transfer notify_transfer;
    
    /* State */
    bool configured;
    bool dtr;
    bool rts;
    
    /* Statistics */
    uint32_t bytes_received;
    uint32_t bytes_transmitted;
    uint32_t rx_errors;
    uint32_t tx_errors;
    
    /* Callbacks */
    void (*rx_callback)(const uint8_t *data, size_t len, void *user_data);
    void (*tx_complete_callback)(void *user_data);
    void (*line_state_callback)(bool dtr, bool rts, void *user_data);
    void *callback_user_data;
};

/* USB Descriptors */
static struct usb_device_descriptor device_descriptor = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DESC_DEVICE,
    .bcdUSB = sys_cpu_to_le16(0x0200),  /* USB 2.0 */
    .bDeviceClass = USB_CDC_CLASS,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = sys_cpu_to_le16(0x1234),
    .idProduct = sys_cpu_to_le16(0x5678),
    .bcdDevice = sys_cpu_to_le16(0x0100),
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/* Configuration Descriptor with CDC ACM */
struct usb_cdc_config_descriptor {
    struct usb_cfg_descriptor config;
    
    /* Communication Interface */
    struct usb_if_descriptor comm_if;
    
    /* CDC Header Functional Descriptor */
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubType;
        uint16_t bcdCDC;
    } __packed cdc_header;
    
    /* CDC Call Management Functional Descriptor */
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubType;
        uint8_t bmCapabilities;
        uint8_t bDataInterface;
    } __packed cdc_call_mgmt;
    
    /* CDC ACM Functional Descriptor */
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubType;
        uint8_t bmCapabilities;
    } __packed cdc_acm;
    
    /* CDC Union Functional Descriptor */
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubType;
        uint8_t bMasterInterface;
        uint8_t bSlaveInterface0;
    } __packed cdc_union;
    
    /* Notification Endpoint */
    struct usb_ep_descriptor notify_ep;
    
    /* Data Interface */
    struct usb_if_descriptor data_if;
    
    /* Data Endpoints */
    struct usb_ep_descriptor data_out_ep;
    struct usb_ep_descriptor data_in_ep;
} __packed;

static struct usb_cdc_config_descriptor config_descriptor = {
    .config = {
        .bLength = sizeof(struct usb_cfg_descriptor),
        .bDescriptorType = USB_DESC_CONFIGURATION,
        .wTotalLength = sys_cpu_to_le16(sizeof(struct usb_cdc_config_descriptor)),
        .bNumInterfaces = 2,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = USB_SCD_SELF_POWERED,
        .bMaxPower = 50,  /* 100mA */
    },
    
    .comm_if = {
        .bLength = sizeof(struct usb_if_descriptor),
        .bDescriptorType = USB_DESC_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = USB_CDC_CLASS,
        .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
        .iInterface = 0,
    },
    
    .cdc_header = {
        .bLength = 5,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_FUNC_DESC_HEADER,
        .bcdCDC = sys_cpu_to_le16(0x0110),
    },
    
    .cdc_call_mgmt = {
        .bLength = 5,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_FUNC_DESC_CALL_MGMT,
        .bmCapabilities = 0x00,
        .bDataInterface = 1,
    },
    
    .cdc_acm = {
        .bLength = 4,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_FUNC_DESC_ACM,
        .bmCapabilities = 0x02,  /* Line coding and serial state */
    },
    
    .cdc_union = {
        .bLength = 5,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_FUNC_DESC_UNION,
        .bMasterInterface = 0,
        .bSlaveInterface0 = 1,
    },
    
    .notify_ep = {
        .bLength = sizeof(struct usb_ep_descriptor),
        .bDescriptorType = USB_DESC_ENDPOINT,
        .bEndpointAddress = 0x82,  /* IN endpoint 2 */
        .bmAttributes = USB_DC_EP_INTERRUPT,
        .wMaxPacketSize = sys_cpu_to_le16(64),
        .bInterval = 10,
    },
    
    .data_if = {
        .bLength = sizeof(struct usb_if_descriptor),
        .bDescriptorType = USB_DESC_INTERFACE,
        .bInterfaceNumber = 1,
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CDC_DATA_CLASS,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    },
    
    .data_out_ep = {
        .bLength = sizeof(struct usb_ep_descriptor),
        .bDescriptorType = USB_DESC_ENDPOINT,
        .bEndpointAddress = 0x01,  /* OUT endpoint 1 */
        .bmAttributes = USB_DC_EP_BULK,
        .wMaxPacketSize = sys_cpu_to_le16(512),
        .bInterval = 0,
    },
    
    .data_in_ep = {
        .bLength = sizeof(struct usb_ep_descriptor),
        .bDescriptorType = USB_DESC_ENDPOINT,
        .bEndpointAddress = 0x81,  /* IN endpoint 1 */
        .bmAttributes = USB_DC_EP_BULK,
        .wMaxPacketSize = sys_cpu_to_le16(512),
        .bInterval = 0,
    },
};

/* String Descriptors */
static struct usb_string_descriptor lang_descriptor = {
    .bLength = 4,
    .bDescriptorType = USB_DESC_STRING,
    .bString = sys_cpu_to_le16(0x0409),  /* English (US) */
};

static struct usb_string_descriptor manufacturer_descriptor = {
    .bLength = 2 + 2 * 16,  /* "Zephyr USB Stack" */
    .bDescriptorType = USB_DESC_STRING,
    .bString = {
        sys_cpu_to_le16('Z'), sys_cpu_to_le16('e'), sys_cpu_to_le16('p'),
        sys_cpu_to_le16('h'), sys_cpu_to_le16('y'), sys_cpu_to_le16('r'),
        sys_cpu_to_le16(' '), sys_cpu_to_le16('U'), sys_cpu_to_le16('S'),
        sys_cpu_to_le16('B'), sys_cpu_to_le16(' '), sys_cpu_to_le16('S'),
        sys_cpu_to_le16('t'), sys_cpu_to_le16('a'), sys_cpu_to_le16('c'),
        sys_cpu_to_le16('k'),
    },
};

static struct usb_string_descriptor product_descriptor = {
    .bLength = 2 + 2 * 18,  /* "USB Serial Device" */
    .bDescriptorType = USB_DESC_STRING,
    .bString = {
        sys_cpu_to_le16('U'), sys_cpu_to_le16('S'), sys_cpu_to_le16('B'),
        sys_cpu_to_le16(' '), sys_cpu_to_le16('S'), sys_cpu_to_le16('e'),
        sys_cpu_to_le16('r'), sys_cpu_to_le16('i'), sys_cpu_to_le16('a'),
        sys_cpu_to_le16('l'), sys_cpu_to_le16(' '), sys_cpu_to_le16('D'),
        sys_cpu_to_le16('e'), sys_cpu_to_le16('v'), sys_cpu_to_le16('i'),
        sys_cpu_to_le16('c'), sys_cpu_to_le16('e'),
    },
};

static struct usb_string_descriptor serial_descriptor = {
    .bLength = 2 + 2 * 12,  /* "123456789ABC" */
    .bDescriptorType = USB_DESC_STRING,
    .bString = {
        sys_cpu_to_le16('1'), sys_cpu_to_le16('2'), sys_cpu_to_le16('3'),
        sys_cpu_to_le16('4'), sys_cpu_to_le16('5'), sys_cpu_to_le16('6'),
        sys_cpu_to_le16('7'), sys_cpu_to_le16('8'), sys_cpu_to_le16('9'),
        sys_cpu_to_le16('A'), sys_cpu_to_le16('B'), sys_cpu_to_le16('C'),
    },
};

static struct usb_string_descriptor *string_descriptors[] = {
    &lang_descriptor,
    &manufacturer_descriptor,
    &product_descriptor,
    &serial_descriptor,
};

/* Transfer completion callbacks */
static void usb_serial_rx_complete(struct usb_stack_transfer *transfer)
{
    struct usb_serial_data *serial = CONTAINER_OF(transfer, struct usb_serial_data, rx_transfer);
    
    if (transfer->status == USB_STACK_TRANSFER_COMPLETE && transfer->actual_length > 0) {
        serial->bytes_received += transfer->actual_length;
        
        /* Notify application */
        if (serial->rx_callback) {
            serial->rx_callback(transfer->buffer, transfer->actual_length, serial->callback_user_data);
        }
        
        LOG_DBG("Received %d bytes", transfer->actual_length);
    } else if (transfer->status == USB_STACK_TRANSFER_ERROR) {
        serial->rx_errors++;
        LOG_ERR("RX transfer error");
    }
    
    /* Resubmit RX transfer if configured */
    if (serial->configured) {
        usb_stack_submit_transfer(&serial->usb_dev, &serial->rx_transfer);
    }
}

static void usb_serial_tx_complete(struct usb_stack_transfer *transfer)
{
    struct usb_serial_data *serial = CONTAINER_OF(transfer, struct usb_serial_data, tx_transfer);
    
    if (transfer->status == USB_STACK_TRANSFER_COMPLETE) {
        serial->bytes_transmitted += transfer->actual_length;
        LOG_DBG("Transmitted %d bytes", transfer->actual_length);
    } else if (transfer->status == USB_STACK_TRANSFER_ERROR) {
        serial->tx_errors++;
        LOG_ERR("TX transfer error");
    }
    
    /* Notify application */
    if (serial->tx_complete_callback) {
        serial->tx_complete_callback(serial->callback_user_data);
    }
    
    k_mutex_unlock(&serial->tx_mutex);
}

/* CDC Class Request Handler */
static int usb_serial_class_handler(struct usb_stack_device *dev, struct usb_setup_packet *setup)
{
    struct usb_serial_data *serial = CONTAINER_OF(dev, struct usb_serial_data, usb_dev);
    
    switch (setup->bRequest) {
    case USB_CDC_SET_LINE_CODING:
        LOG_DBG("Set Line Coding");
        if (setup->wLength == sizeof(struct cdc_line_coding)) {
            /* Receive line coding data */
            return usb_stack_control_response(dev, (uint8_t *)&serial->line_coding, 
                                            sizeof(serial->line_coding));
        }
        break;
        
    case USB_CDC_GET_LINE_CODING:
        LOG_DBG("Get Line Coding");
        return usb_stack_control_response(dev, (uint8_t *)&serial->line_coding, 
                                        sizeof(serial->line_coding));
        
    case USB_CDC_SET_CONTROL_LINE_STATE:
        LOG_DBG("Set Control Line State: 0x%04x", setup->wValue);
        serial->control_line_state = setup->wValue;
        serial->dtr = (setup->wValue & BIT(0)) != 0;
        serial->rts = (setup->wValue & BIT(1)) != 0;
        
        /* Notify application */
        if (serial->line_state_callback) {
            serial->line_state_callback(serial->dtr, serial->rts, serial->callback_user_data);
        }
        
        return usb_stack_control_response(dev, NULL, 0);
        
    case USB_CDC_SEND_BREAK:
        LOG_DBG("Send Break");
        return usb_stack_control_response(dev, NULL, 0);
        
    default:
        LOG_WRN("Unsupported CDC request: 0x%02x", setup->bRequest);
        break;
    }
    
    return -EINVAL;
}

/* USB Event Handler */
static void usb_serial_event_handler(struct usb_stack_device *dev, usb_stack_event_type_t event, void *data)
{
    struct usb_serial_data *serial = CONTAINER_OF(dev, struct usb_serial_data, usb_dev);
    
    switch (event) {
    case USB_STACK_EVENT_RESET:
        LOG_INF("USB Reset");
        serial->configured = false;
        break;
        
    case USB_STACK_EVENT_CONNECT:
        LOG_INF("USB Connected");
        break;
        
    case USB_STACK_EVENT_DISCONNECT:
        LOG_INF("USB Disconnected");
        serial->configured = false;
        break;
        
    case USB_STACK_EVENT_SUSPEND:
        LOG_INF("USB Suspended");
        break;
        
    case USB_STACK_EVENT_RESUME:
        LOG_INF("USB Resumed");
        break;
        
    default:
        break;
    }
}

/* Configuration Handler */
static int usb_serial_configure(struct usb_serial_data *serial)
{
    int ret;
    
    LOG_INF("Configuring USB Serial");
    
    /* Configure data endpoints */
    ret = usb_stack_configure_endpoint(&serial->usb_dev, serial->data_out_ep,
                                      USB_STACK_EP_TYPE_BULK, 512, 0);
    if (ret) {
        LOG_ERR("Failed to configure OUT endpoint: %d", ret);
        return ret;
    }
    
    ret = usb_stack_configure_endpoint(&serial->usb_dev, serial->data_in_ep,
                                      USB_STACK_EP_TYPE_BULK, 512, 0);
    if (ret) {
        LOG_ERR("Failed to configure IN endpoint: %d", ret);
        return ret;
    }
    
    /* Configure notification endpoint */
    ret = usb_stack_configure_endpoint(&serial->usb_dev, serial->notify_ep,
                                      USB_STACK_EP_TYPE_INTERRUPT, 64, 10);
    if (ret) {
        LOG_ERR("Failed to configure notify endpoint: %d", ret);
        return ret;
    }
    
    /* Initialize transfers */
    usb_stack_init_transfer(&serial->rx_transfer, serial->data_out_ep,
                           serial->rx_buffer, 512, usb_serial_rx_complete, NULL);
    
    usb_stack_init_transfer(&serial->tx_transfer, serial->data_in_ep,
                           serial->tx_buffer, 512, usb_serial_tx_complete, NULL);
    
    /* Submit initial RX transfer */
    ret = usb_stack_submit_transfer(&serial->usb_dev, &serial->rx_transfer);
    if (ret) {
        LOG_ERR("Failed to submit RX transfer: %d", ret);
        return ret;
    }
    
    serial->configured = true;
    
    LOG_INF("USB Serial configured successfully");
    return 0;
}

/* Public API */

int usb_serial_init(struct usb_serial_data *serial)
{
    if (!serial) {
        return -EINVAL;
    }
    
    LOG_INF("Initializing USB Serial");
    
    /* Initialize data structure */
    memset(serial, 0, sizeof(*serial));
    
    /* Set default line coding */
    serial->line_coding.dwDTERate = sys_cpu_to_le32(115200);
    serial->line_coding.bCharFormat = 0;  /* 1 stop bit */
    serial->line_coding.bParityType = 0;  /* No parity */
    serial->line_coding.bDataBits = 8;    /* 8 data bits */
    
    /* Set endpoint addresses */
    serial->data_out_ep = 0x01;
    serial->data_in_ep = 0x81;
    serial->notify_ep = 0x82;
    
    /* Allocate buffers */
    serial->rx_buffer = k_malloc(512);
    serial->tx_buffer = k_malloc(512);
    serial->notify_buffer = k_malloc(64);
    
    if (!serial->rx_buffer || !serial->tx_buffer || !serial->notify_buffer) {
        LOG_ERR("Failed to allocate buffers");
        return -ENOMEM;
    }
    
    /* Initialize synchronization */
    k_mutex_init(&serial->tx_mutex);
    k_fifo_init(&serial->rx_fifo);
    k_fifo_init(&serial->tx_fifo);
    
    /* Configure USB device */
    struct usb_stack_device_config usb_config = {
        .device_desc = &device_descriptor,
        .config_desc = (struct usb_cfg_descriptor *)&config_descriptor,
        .string_descs = string_descriptors,
        .num_strings = ARRAY_SIZE(string_descriptors),
        .event_callback = usb_serial_event_handler,
        .setup_callback = usb_serial_class_handler,
        .self_powered = true,
        .remote_wakeup = false,
        .max_power = 50,
    };
    
    /* Initialize USB stack */
    int ret = usb_stack_init(&serial->usb_dev, &usb_config);
    if (ret) {
        LOG_ERR("Failed to initialize USB stack: %d", ret);
        return ret;
    }
    
    LOG_INF("USB Serial initialized successfully");
    return 0;
}

int usb_serial_enable(struct usb_serial_data *serial)
{
    if (!serial) {
        return -EINVAL;
    }
    
    LOG_INF("Enabling USB Serial");
    
    int ret = usb_stack_enable(&serial->usb_dev);
    if (ret) {
        LOG_ERR("Failed to enable USB stack: %d", ret);
        return ret;
    }
    
    /* Configure endpoints when device is configured */
    ret = usb_serial_configure(serial);
    if (ret) {
        LOG_ERR("Failed to configure serial: %d", ret);
        return ret;
    }
    
    return 0;
}

int usb_serial_disable(struct usb_serial_data *serial)
{
    if (!serial) {
        return -EINVAL;
    }
    
    LOG_INF("Disabling USB Serial");
    
    serial->configured = false;
    
    return usb_stack_disable(&serial->usb_dev);
}

int usb_serial_write(struct usb_serial_data *serial, const uint8_t *data, size_t len)
{
    if (!serial || !data || len == 0) {
        return -EINVAL;
    }
    
    if (!serial->configured) {
        return -ENOTCONN;
    }
    
    if (len > 512) {
        len = 512;  /* Limit to buffer size */
    }
    
    /* Wait for previous transfer to complete */
    k_mutex_lock(&serial->tx_mutex, K_FOREVER);
    
    /* Copy data to TX buffer */
    memcpy(serial->tx_buffer, data, len);
    serial->tx_transfer.length = len;
    
    /* Submit transfer */
    int ret = usb_stack_submit_transfer(&serial->usb_dev, &serial->tx_transfer);
    if (ret) {
        k_mutex_unlock(&serial->tx_mutex);
        LOG_ERR("Failed to submit TX transfer: %d", ret);
        return ret;
    }
    
    return len;
}

int usb_serial_register_callbacks(struct usb_serial_data *serial,
                                 void (*rx_callback)(const uint8_t *data, size_t len, void *user_data),
                                 void (*tx_complete_callback)(void *user_data),
                                 void (*line_state_callback)(bool dtr, bool rts, void *user_data),
                                 void *user_data)
{
    if (!serial) {
        return -EINVAL;
    }
    
    serial->rx_callback = rx_callback;
    serial->tx_complete_callback = tx_complete_callback;
    serial->line_state_callback = line_state_callback;
    serial->callback_user_data = user_data;
    
    return 0;
}

void usb_serial_get_statistics(struct usb_serial_data *serial,
                              uint32_t *bytes_rx, uint32_t *bytes_tx,
                              uint32_t *rx_errors, uint32_t *tx_errors)
{
    if (!serial) {
        return;
    }
    
    if (bytes_rx) *bytes_rx = serial->bytes_received;
    if (bytes_tx) *bytes_tx = serial->bytes_transmitted;
    if (rx_errors) *rx_errors = serial->rx_errors;
    if (tx_errors) *tx_errors = serial->tx_errors;
}

/* Global instance for simple usage */
static struct usb_serial_data g_usb_serial;

int usb_serial_simple_init(void)
{
    return usb_serial_init(&g_usb_serial);
}

int usb_serial_simple_enable(void)
{
    return usb_serial_enable(&g_usb_serial);
}

int usb_serial_simple_write(const uint8_t *data, size_t len)
{
    return usb_serial_write(&g_usb_serial, data, len);
}

int usb_serial_simple_register_rx_callback(void (*callback)(const uint8_t *data, size_t len, void *user_data),
                                          void *user_data)
{
    return usb_serial_register_callbacks(&g_usb_serial, callback, NULL, NULL, user_data);
}
