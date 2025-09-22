/*
 * USB Stack Initialization Module
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "../include/usb_stack.h"
#include "../include/usb_stack_config.h"
#include "../include/usb_stack_types.h"

LOG_MODULE_REGISTER(usb_stack_init, CONFIG_USB_STACK_LOG_LEVEL);

/* Global USB stack instance */
static struct usb_stack_device g_usb_stack_device;
static bool g_usb_stack_initialized = false;

/* Device tree nodes */
#define USB_CONTROLLER_NODE DT_NODELABEL(usb_controller)
#define USB_PHY_NODE DT_NODELABEL(usb_phy)
#define TYPEC_MANAGER_NODE DT_NODELABEL(typec_manager)

/* Forward declarations */
static int usb_stack_hw_init(void);
static int usb_stack_phy_init(void);
static int usb_stack_typec_init(void);
static int usb_stack_core_init(void);

/**
 * @brief Initialize USB stack hardware components
 */
static int usb_stack_hw_init(void)
{
    int ret;
    
    LOG_INF("Initializing USB stack hardware");
    
    /* Initialize DWC3 controller - get device from device tree */
    const struct device *dwc3_dev = DEVICE_DT_GET_ANY(synopsys_dwc3);
    if (!dwc3_dev) {
        LOG_ERR("DWC3 controller device not found");
        return -ENODEV;
    }
    
    /* DWC3 controller is initialized by its own init function */
    LOG_DBG("DWC3 controller device found");
    
    LOG_DBG("DWC3 controller initialized successfully");
    return 0;
}

/**
 * @brief Initialize USB PHY components
 */
static int usb_stack_phy_init(void)
{
    int ret;
    
    LOG_INF("Initializing USB PHY");
    
    /* Initialize Qualcomm PHY - get device from device tree */
    const struct device *phy_dev = DEVICE_DT_GET_ANY(qcom_usb_phy);
    if (!phy_dev) {
        LOG_ERR("Qualcomm PHY device not found");
        return -ENODEV;
    }
    
    /* PHY is initialized by its own init function */
    LOG_DBG("Qualcomm PHY device found");
    
    LOG_DBG("Qualcomm PHY initialized successfully");
    return 0;
}

/**
 * @brief Initialize Type-C manager
 */
static int usb_stack_typec_init(void)
{
    int ret;
    
    LOG_INF("Initializing Type-C manager");
    
    /* Initialize Type-C manager - get device from device tree */
    const struct device *typec_dev = DEVICE_DT_GET_ANY(typec_manager);
    if (!typec_dev) {
        LOG_ERR("Type-C manager device not found");
        return -ENODEV;
    }
    
    /* Type-C manager is initialized by its own init function */
    LOG_DBG("Type-C manager device found");
    
    LOG_DBG("Type-C manager initialized successfully");
    return 0;
}

/**
 * @brief Initialize USB stack core
 */
static int usb_stack_core_init(void)
{
    int ret;
    
    LOG_INF("Initializing USB stack core");
    
    /* Initialize USB stack core */
    ret = usb_stack_init(&g_usb_stack_device, g_usb_stack_device.config);
    if (ret) {
        LOG_ERR("Failed to initialize USB stack core: %d", ret);
        return ret;
    }
    
    LOG_DBG("USB stack core initialized successfully");
    return 0;
}

/**
 * @brief Main USB stack initialization function
 */
static int usb_stack_system_init(void)
{
    int ret;
    
    LOG_INF("Starting USB stack initialization");
    LOG_INF("USB Stack Version: %d.%d.%d", 
            USB_STACK_VERSION_MAJOR, 
            USB_STACK_VERSION_MINOR, 
            USB_STACK_VERSION_PATCH);
    
    /* Check if device tree nodes are available */
#if DT_NODE_EXISTS(USB_CONTROLLER_NODE)
    LOG_DBG("USB controller node found in device tree");
#else
    LOG_WRN("USB controller node not found in device tree");
#endif

#if DT_NODE_EXISTS(USB_PHY_NODE)
    LOG_DBG("USB PHY node found in device tree");
#else
    LOG_WRN("USB PHY node not found in device tree");
#endif

#if DT_NODE_EXISTS(TYPEC_MANAGER_NODE)
    LOG_DBG("Type-C manager node found in device tree");
#else
    LOG_WRN("Type-C manager node not found in device tree");
#endif
    
    /* Initialize global USB stack device structure */
    memset(&g_usb_stack_device, 0, sizeof(g_usb_stack_device));
    
    /* Set default configuration - config is a pointer, so we need to allocate and set it */
    static struct usb_stack_device_config default_config = {0};
    default_config.self_powered = true;
    default_config.remote_wakeup = false;
    default_config.max_power = 250; /* 500mA in 2mA units */
    g_usb_stack_device.config = &default_config;
    
    /* Initialize hardware components in order */
    
    /* Step 1: Initialize PHY first */
    if (IS_ENABLED(CONFIG_USB_STACK_QCOM_PHY)) {
        ret = usb_stack_phy_init();
        if (ret) {
            LOG_ERR("PHY initialization failed: %d", ret);
            return ret;
        }
    }
    
    /* Step 2: Initialize USB controller */
    if (IS_ENABLED(CONFIG_USB_STACK_DWC3_CONTROLLER)) {
        ret = usb_stack_hw_init();
        if (ret) {
            LOG_ERR("Hardware initialization failed: %d", ret);
            return ret;
        }
    }
    
    /* Step 3: Initialize Type-C manager */
    if (IS_ENABLED(CONFIG_USB_STACK_TYPEC_SUPPORT)) {
        ret = usb_stack_typec_init();
        if (ret) {
            LOG_ERR("Type-C initialization failed: %d", ret);
            return ret;
        }
    }
    
    /* Step 4: Initialize USB stack core */
    ret = usb_stack_core_init();
    if (ret) {
        LOG_ERR("USB stack core initialization failed: %d", ret);
        return ret;
    }
    
    /* Mark as initialized */
    g_usb_stack_initialized = true;
    
    LOG_INF("USB stack initialization completed successfully");
    
    /* Print configuration summary */
    LOG_INF("USB Stack Configuration:");
    LOG_INF("  Self Powered: %s", g_usb_stack_device.config->self_powered ? "Yes" : "No");
    LOG_INF("  Remote Wakeup: %s", g_usb_stack_device.config->remote_wakeup ? "Yes" : "No");
    LOG_INF("  Max Power: %d mA", g_usb_stack_device.config->max_power * 2);
    
    return 0;
}

/**
 * @brief Get global USB stack device instance
 */
struct usb_stack_device *usb_stack_get_device(void)
{
    if (!g_usb_stack_initialized) {
        LOG_ERR("USB stack not initialized");
        return NULL;
    }
    
    return &g_usb_stack_device;
}

/**
 * @brief Check if USB stack is initialized
 */
bool usb_stack_is_initialized(void)
{
    return g_usb_stack_initialized;
}

/**
 * @brief Deinitialize USB stack (for cleanup)
 */
int usb_stack_deinit(struct usb_stack_device *dev)
{
    int ret = 0;
    
    if (!dev) {
        LOG_ERR("Invalid device pointer");
        return USB_STACK_ERROR_INVALID_PARAM;
    }
    
    if (!g_usb_stack_initialized) {
        LOG_WRN("USB stack not initialized, nothing to deinitialize");
        return 0;
    }
    
    LOG_INF("Deinitializing USB stack");
    
    /* Deinitialize in reverse order */
    
    /* Stop USB stack core */
    if (dev->state != USB_STACK_STATE_DETACHED) {
        usb_stack_disable(dev);
    }
    
    /* Deinitialize Type-C manager */
    if (IS_ENABLED(CONFIG_USB_STACK_TYPEC_SUPPORT)) {
        LOG_DBG("Type-C manager deinitialized");
    }
    
    /* Deinitialize USB controller */
    if (IS_ENABLED(CONFIG_USB_STACK_DWC3_CONTROLLER)) {
        LOG_DBG("DWC3 controller deinitialized");
    }
    
    /* Deinitialize PHY */
    if (IS_ENABLED(CONFIG_USB_STACK_QCOM_PHY)) {
        LOG_DBG("Qualcomm PHY deinitialized");
    }
    
    /* Clear device state */
    dev->state = USB_STACK_STATE_DETACHED;
    dev->speed = USB_STACK_SPEED_UNKNOWN;
    dev->address = 0;
    dev->configuration = 0;
    
    /* If this is the global device, clear global state */
    if (dev == &g_usb_stack_device) {
        g_usb_stack_initialized = false;
    }
    
    LOG_INF("USB stack deinitialization completed");
    
    return ret;
}

/**
 * @brief USB stack system initialization hook
 * 
 * This function is called during system initialization at the priority
 * specified by CONFIG_USB_STACK_INIT_PRIORITY.
 */
SYS_INIT(usb_stack_system_init, POST_KERNEL, CONFIG_USB_STACK_INIT_PRIORITY);

/**
 * @brief Simple initialization API for applications
 */
int usb_stack_simple_init(void)
{
    /* The actual initialization is done by SYS_INIT */
    if (!g_usb_stack_initialized) {
        LOG_ERR("USB stack system initialization failed");
        return -ENODEV;
    }
    
    LOG_INF("USB stack ready for use");
    return 0;
}

/**
 * @brief Get USB stack version information
 */
void usb_stack_get_version(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (major) *major = USB_STACK_VERSION_MAJOR;
    if (minor) *minor = USB_STACK_VERSION_MINOR;
    if (patch) *patch = USB_STACK_VERSION_PATCH;
}

/**
 * @brief Get USB stack build information
 */
const char *usb_stack_get_build_info(void)
{
    return "USB Stack v" STRINGIFY(USB_STACK_VERSION_MAJOR) "." 
           STRINGIFY(USB_STACK_VERSION_MINOR) "." 
           STRINGIFY(USB_STACK_VERSION_PATCH) 
           " - Built " __DATE__ " " __TIME__;
}

/**
 * @brief Runtime configuration update
 */
int usb_stack_update_config(const struct usb_stack_device_config *new_config)
{
    if (!g_usb_stack_initialized) {
        LOG_ERR("USB stack not initialized");
        return -ENODEV;
    }
    
    if (!new_config) {
        LOG_ERR("Invalid configuration pointer");
        return -EINVAL;
    }
    
    /* Validate configuration parameters */
    if (new_config->max_power > 500) {
        LOG_ERR("Invalid max_power: %d (must be <= 500mA)", new_config->max_power * 2);
        return -EINVAL;
    }
    
    /* Update configuration - since config is a pointer, we need to copy the content */
    static struct usb_stack_device_config updated_config;
    memcpy(&updated_config, new_config, sizeof(struct usb_stack_device_config));
    g_usb_stack_device.config = &updated_config;
    
    LOG_INF("USB stack configuration updated");
    return 0;
}

/**
 * @brief Get current USB stack configuration
 */
int usb_stack_get_config(struct usb_stack_device_config *config)
{
    if (!g_usb_stack_initialized) {
        LOG_ERR("USB stack not initialized");
        return -ENODEV;
    }
    
    if (!config) {
        LOG_ERR("Invalid configuration pointer");
        return -EINVAL;
    }
    
    memcpy(config, g_usb_stack_device.config, sizeof(struct usb_stack_device_config));
    return 0;
}
