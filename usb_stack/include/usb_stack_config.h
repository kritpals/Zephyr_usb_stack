/*
 * USB Stack Configuration Header
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USB_STACK_CONFIG_H
#define USB_STACK_CONFIG_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USB Stack Version */
#define USB_STACK_VERSION_MAJOR    1
#define USB_STACK_VERSION_MINOR    0
#define USB_STACK_VERSION_PATCH    0

/* USB Standards Support */
#define USB_STACK_USB20_SUPPORT    1
#define USB_STACK_USB30_SUPPORT    1
#define USB_STACK_USB31_SUPPORT    1

/* Controller Support */
#define USB_STACK_DWC3_SUPPORT     1
#define USB_STACK_SYNOPSYS_M31     1

/* PHY Support */
#define USB_STACK_QCOM_PHY_SUPPORT 1
#define USB_STACK_QMP_PHY_SUPPORT  1
#define USB_STACK_QUSB2_PHY_SUPPORT 1

/* Type-C Support */
#define USB_STACK_TYPEC_SUPPORT    1
#define USB_STACK_PD_SUPPORT       1

/* Device Mode Configuration */
#define USB_STACK_DEVICE_MODE      1
#define USB_STACK_HOST_MODE        0  /* Not implemented in this version */

/* Endpoint Configuration */
#define USB_STACK_MAX_ENDPOINTS    16
#define USB_STACK_CONTROL_EP       0
#define USB_STACK_BULK_EP_COUNT    8
#define USB_STACK_INT_EP_COUNT     4
#define USB_STACK_ISO_EP_COUNT     2

/* Interface Configuration */
#define USB_STACK_MAX_INTERFACES   8
#define USB_STACK_MAX_ALT_SETTINGS 4

/* Buffer Configuration */
#define USB_STACK_CTRL_BUF_SIZE    512
#define USB_STACK_BULK_BUF_SIZE    1024
#define USB_STACK_INT_BUF_SIZE     64
#define USB_STACK_ISO_BUF_SIZE     1024

/* Transfer Configuration */
#define USB_STACK_MAX_CONCURRENT_TRANSFERS  32
#define USB_STACK_MAX_TRANSFER_SIZE         65536

/* DMA Configuration */
#define USB_STACK_DMA_SUPPORT      1
#define USB_STACK_DMA_COHERENT     1

/* Power Management */
#define USB_STACK_PM_SUPPORT       1
#define USB_STACK_REMOTE_WAKEUP    1

/* Debug Configuration */
#define USB_STACK_DEBUG_LEVEL      2  /* 0=None, 1=Error, 2=Info, 3=Debug */

/* Thread Configuration */
#define USB_STACK_THREAD_PRIORITY  K_PRIO_COOP(4)
#define USB_STACK_THREAD_STACK_SIZE 2048

/* Timeout Configuration */
#define USB_STACK_ENUM_TIMEOUT_MS  5000
#define USB_STACK_XFER_TIMEOUT_MS  1000
#define USB_STACK_RESET_TIMEOUT_MS 100

/* Zephyr Configuration Constants */
//#define CONFIG_USB_STACK_LOG_LEVEL         LOG_LEVEL_INF
#define CONFIG_USB_STACK_INIT_PRIORITY     50
#define CONFIG_USB_STACK_QCOM_PHY          1
#define CONFIG_USB_STACK_DWC3_CONTROLLER   1
#define CONFIG_USB_STACK_TYPEC_SUPPORT     1

#ifdef __cplusplus
}
#endif

#endif /* USB_STACK_CONFIG_H */
