/*
 * Qualcomm USB Retimer Support
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "../include/usb_stack.h"
#include "../include/usb_stack_types.h"

LOG_MODULE_REGISTER(qcom_retimer, CONFIG_USB_STACK_LOG_LEVEL);

/* Qualcomm USB Retimer Register Definitions */
#define QCOM_RETIMER_DEVICE_ID_REG          0x00
#define QCOM_RETIMER_REVISION_REG           0x01
#define QCOM_RETIMER_GENERAL_CFG_REG        0x02
#define QCOM_RETIMER_CONNECTION_STATE_REG   0x03
#define QCOM_RETIMER_USB_CFG_REG            0x10
#define QCOM_RETIMER_DP_CFG_REG             0x20
#define QCOM_RETIMER_ORIENTATION_REG        0x30
#define QCOM_RETIMER_POWER_STATE_REG        0x40

/* Retimer Configuration Values */
#define QCOM_RETIMER_USB3_MODE              0x01
#define QCOM_RETIMER_USB2_MODE              0x02
#define QCOM_RETIMER_DP_MODE                0x04
#define QCOM_RETIMER_USB3_DP_MODE           0x05

#define QCOM_RETIMER_ORIENTATION_NORMAL     0x00
#define QCOM_RETIMER_ORIENTATION_FLIPPED    0x01

#define QCOM_RETIMER_POWER_ON               0x01
#define QCOM_RETIMER_POWER_OFF              0x00

/**
 * @brief Initialize Qualcomm USB retimer
 */
int qcom_retimer_init(struct qcom_phy_data *phy)
{
    int ret;
    uint8_t device_id, revision;
    
    if (!phy || !phy->retimer_dev) {
        LOG_ERR("Invalid retimer device");
        return -EINVAL;
    }
    
    LOG_INF("Initializing Qualcomm USB retimer");
    
    /* Read device ID to verify retimer presence */
    ret = i2c_reg_read_byte(phy->retimer_dev, phy->retimer_addr, 
                           QCOM_RETIMER_DEVICE_ID_REG, &device_id);
    if (ret) {
        LOG_ERR("Failed to read retimer device ID: %d", ret);
        return ret;
    }
    
    ret = i2c_reg_read_byte(phy->retimer_dev, phy->retimer_addr,
                           QCOM_RETIMER_REVISION_REG, &revision);
    if (ret) {
        LOG_ERR("Failed to read retimer revision: %d", ret);
        return ret;
    }
    
    LOG_INF("Retimer detected: ID=0x%02x, Revision=0x%02x", device_id, revision);
    
    /* Configure retimer for USB3 + DP mode */
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_GENERAL_CFG_REG, QCOM_RETIMER_USB3_DP_MODE);
    if (ret) {
        LOG_ERR("Failed to configure retimer mode: %d", ret);
        return ret;
    }
    
    /* Set default orientation */
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_ORIENTATION_REG, QCOM_RETIMER_ORIENTATION_NORMAL);
    if (ret) {
        LOG_ERR("Failed to set retimer orientation: %d", ret);
        return ret;
    }
    
    /* Power on retimer */
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_POWER_STATE_REG, QCOM_RETIMER_POWER_ON);
    if (ret) {
        LOG_ERR("Failed to power on retimer: %d", ret);
        return ret;
    }
    
    /* Wait for retimer to stabilize */
    k_msleep(10);
    
    LOG_DBG("Qualcomm USB retimer initialized successfully");
    return 0;
}

/**
 * @brief Configure retimer for specific USB mode
 */
int qcom_retimer_set_mode(struct qcom_phy_data *phy, qcom_retimer_mode_t mode)
{
    int ret;
    uint8_t config_val;
    
    if (!phy || !phy->retimer_dev) {
        LOG_ERR("Invalid retimer device");
        return -EINVAL;
    }
    
    switch (mode) {
    case QCOM_RETIMER_MODE_USB2:
        config_val = QCOM_RETIMER_USB2_MODE;
        LOG_INF("Setting retimer to USB2 mode");
        break;
        
    case QCOM_RETIMER_MODE_USB3:
        config_val = QCOM_RETIMER_USB3_MODE;
        LOG_INF("Setting retimer to USB3 mode");
        break;
        
    case QCOM_RETIMER_MODE_DP:
        config_val = QCOM_RETIMER_DP_MODE;
        LOG_INF("Setting retimer to DisplayPort mode");
        break;
        
    case QCOM_RETIMER_MODE_USB3_DP:
        config_val = QCOM_RETIMER_USB3_DP_MODE;
        LOG_INF("Setting retimer to USB3 + DisplayPort mode");
        break;
        
    default:
        LOG_ERR("Invalid retimer mode: %d", mode);
        return -EINVAL;
    }
    
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_GENERAL_CFG_REG, config_val);
    if (ret) {
        LOG_ERR("Failed to set retimer mode: %d", ret);
        return ret;
    }
    
    /* Wait for mode change to take effect */
    k_msleep(5);
    
    return 0;
}

/**
 * @brief Set retimer orientation
 */
int qcom_retimer_set_orientation(struct qcom_phy_data *phy, bool flipped)
{
    int ret;
    uint8_t orientation_val;
    
    if (!phy || !phy->retimer_dev) {
        LOG_ERR("Invalid retimer device");
        return -EINVAL;
    }
    
    orientation_val = flipped ? QCOM_RETIMER_ORIENTATION_FLIPPED : QCOM_RETIMER_ORIENTATION_NORMAL;
    
    LOG_INF("Setting retimer orientation: %s", flipped ? "flipped" : "normal");
    
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_ORIENTATION_REG, orientation_val);
    if (ret) {
        LOG_ERR("Failed to set retimer orientation: %d", ret);
        return ret;
    }
    
    /* Wait for orientation change to take effect */
    k_msleep(2);
    
    return 0;
}

/**
 * @brief Get retimer connection state
 */
int qcom_retimer_get_connection_state(struct qcom_phy_data *phy, uint8_t *state)
{
    int ret;
    
    if (!phy || !phy->retimer_dev || !state) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    ret = i2c_reg_read_byte(phy->retimer_dev, phy->retimer_addr,
                           QCOM_RETIMER_CONNECTION_STATE_REG, state);
    if (ret) {
        LOG_ERR("Failed to read retimer connection state: %d", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Power down retimer
 */
int qcom_retimer_power_down(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy || !phy->retimer_dev) {
        LOG_ERR("Invalid retimer device");
        return -EINVAL;
    }
    
    LOG_INF("Powering down retimer");
    
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_POWER_STATE_REG, QCOM_RETIMER_POWER_OFF);
    if (ret) {
        LOG_ERR("Failed to power down retimer: %d", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Configure retimer for USB3 SuperSpeed operation
 */
int qcom_retimer_configure_usb3(struct qcom_phy_data *phy, usb_speed_t speed)
{
    int ret;
    uint8_t usb_config = 0;
    
    if (!phy || !phy->retimer_dev) {
        LOG_ERR("Invalid retimer device");
        return -EINVAL;
    }
    
    /* Configure USB3 specific settings based on speed */
    switch (speed) {
    case USB_SPEED_SUPER:
        usb_config = 0x10; /* USB3.0 SuperSpeed */
        LOG_INF("Configuring retimer for USB3.0 SuperSpeed");
        break;
        
    case USB_SPEED_SUPER_PLUS:
        usb_config = 0x20; /* USB3.1 SuperSpeed+ */
        LOG_INF("Configuring retimer for USB3.1 SuperSpeed+");
        break;
        
    default:
        LOG_WRN("Unsupported USB3 speed: %d", speed);
        usb_config = 0x10; /* Default to USB3.0 */
        break;
    }
    
    ret = i2c_reg_write_byte(phy->retimer_dev, phy->retimer_addr,
                            QCOM_RETIMER_USB_CFG_REG, usb_config);
    if (ret) {
        LOG_ERR("Failed to configure retimer for USB3: %d", ret);
        return ret;
    }
    
    /* Wait for configuration to take effect */
    k_msleep(5);
    
    return 0;
}

/**
 * @brief Reset retimer
 */
int qcom_retimer_reset(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy) {
        LOG_ERR("Invalid PHY data");
        return -EINVAL;
    }
    
    LOG_INF("Resetting retimer");
    
    /* Assert reset if GPIO is available */
    if (phy->retimer_reset_gpio.port) {
        ret = gpio_pin_set_dt(&phy->retimer_reset_gpio, 1);
        if (ret) {
            LOG_ERR("Failed to assert retimer reset: %d", ret);
            return ret;
        }
        
        k_msleep(1);
        
        ret = gpio_pin_set_dt(&phy->retimer_reset_gpio, 0);
        if (ret) {
            LOG_ERR("Failed to deassert retimer reset: %d", ret);
            return ret;
        }
        
        /* Wait for reset to complete */
        k_msleep(10);
    }
    
    return 0;
}

/**
 * @brief Deinitialize retimer
 */
int qcom_retimer_deinit(struct qcom_phy_data *phy)
{
    if (!phy) {
        LOG_ERR("Invalid PHY data");
        return -EINVAL;
    }
    
    LOG_INF("Deinitializing retimer");
    
    /* Power down retimer */
    qcom_retimer_power_down(phy);
    
    return 0;
}
