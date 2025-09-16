/*
 * Qualcomm USB Repeater Support
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

LOG_MODULE_REGISTER(qcom_repeater, CONFIG_USB_STACK_LOG_LEVEL);

/* Qualcomm USB Repeater Register Definitions */
#define QCOM_REPEATER_DEVICE_ID_REG         0x00
#define QCOM_REPEATER_REVISION_REG          0x01
#define QCOM_REPEATER_CONTROL_REG           0x02
#define QCOM_REPEATER_STATUS_REG            0x03
#define QCOM_REPEATER_CONFIG_REG            0x04
#define QCOM_REPEATER_SIGNAL_DETECT_REG     0x05
#define QCOM_REPEATER_LOSS_OF_SIGNAL_REG    0x06
#define QCOM_REPEATER_CDR_CONTROL_REG       0x07

/* Repeater Control Values */
#define QCOM_REPEATER_ENABLE                0x01
#define QCOM_REPEATER_DISABLE               0x00
#define QCOM_REPEATER_RESET                 0x80

/* Configuration Values */
#define QCOM_REPEATER_USB3_MODE             0x01
#define QCOM_REPEATER_USB2_MODE             0x02
#define QCOM_REPEATER_AUTO_MODE             0x03

/* Signal Detection Thresholds */
#define QCOM_REPEATER_SIGNAL_DETECT_HIGH    0x0F
#define QCOM_REPEATER_SIGNAL_DETECT_MED     0x07
#define QCOM_REPEATER_SIGNAL_DETECT_LOW     0x03

/* CDR (Clock Data Recovery) Settings */
#define QCOM_REPEATER_CDR_ENABLE            0x01
#define QCOM_REPEATER_CDR_DISABLE           0x00

/**
 * @brief Initialize Qualcomm USB repeater
 */
int qcom_repeater_init(struct qcom_phy_data *phy)
{
    int ret;
    uint8_t device_id, revision;
    
    if (!phy || !phy->repeater_dev) {
        LOG_ERR("Invalid repeater device");
        return -EINVAL;
    }
    
    LOG_INF("Initializing Qualcomm USB repeater");
    
    /* Read device ID to verify repeater presence */
    ret = i2c_reg_read_byte(phy->repeater_dev, phy->repeater_addr,
                           QCOM_REPEATER_DEVICE_ID_REG, &device_id);
    if (ret) {
        LOG_ERR("Failed to read repeater device ID: %d", ret);
        return ret;
    }
    
    ret = i2c_reg_read_byte(phy->repeater_dev, phy->repeater_addr,
                           QCOM_REPEATER_REVISION_REG, &revision);
    if (ret) {
        LOG_ERR("Failed to read repeater revision: %d", ret);
        return ret;
    }
    
    LOG_INF("Repeater detected: ID=0x%02x, Revision=0x%02x", device_id, revision);
    
    /* Reset repeater */
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CONTROL_REG, QCOM_REPEATER_RESET);
    if (ret) {
        LOG_ERR("Failed to reset repeater: %d", ret);
        return ret;
    }
    
    /* Wait for reset to complete */
    k_msleep(10);
    
    /* Configure repeater for auto mode (USB2/USB3 detection) */
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CONFIG_REG, QCOM_REPEATER_AUTO_MODE);
    if (ret) {
        LOG_ERR("Failed to configure repeater mode: %d", ret);
        return ret;
    }
    
    /* Set signal detection threshold */
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_SIGNAL_DETECT_REG, QCOM_REPEATER_SIGNAL_DETECT_MED);
    if (ret) {
        LOG_ERR("Failed to set repeater signal detection: %d", ret);
        return ret;
    }
    
    /* Enable CDR */
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CDR_CONTROL_REG, QCOM_REPEATER_CDR_ENABLE);
    if (ret) {
        LOG_ERR("Failed to enable repeater CDR: %d", ret);
        return ret;
    }
    
    /* Enable repeater */
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CONTROL_REG, QCOM_REPEATER_ENABLE);
    if (ret) {
        LOG_ERR("Failed to enable repeater: %d", ret);
        return ret;
    }
    
    /* Wait for repeater to stabilize */
    k_msleep(5);
    
    LOG_DBG("Qualcomm USB repeater initialized successfully");
    return 0;
}

/**
 * @brief Set repeater mode
 */
int qcom_repeater_set_mode(struct qcom_phy_data *phy, qcom_repeater_mode_t mode)
{
    int ret;
    uint8_t config_val;
    
    if (!phy || !phy->repeater_dev) {
        LOG_ERR("Invalid repeater device");
        return -EINVAL;
    }
    
    switch (mode) {
    case QCOM_REPEATER_MODE_USB2:
        config_val = QCOM_REPEATER_USB2_MODE;
        LOG_INF("Setting repeater to USB2 mode");
        break;
        
    case QCOM_REPEATER_MODE_USB3:
        config_val = QCOM_REPEATER_USB3_MODE;
        LOG_INF("Setting repeater to USB3 mode");
        break;
        
    case QCOM_REPEATER_MODE_AUTO:
        config_val = QCOM_REPEATER_AUTO_MODE;
        LOG_INF("Setting repeater to auto detection mode");
        break;
        
    default:
        LOG_ERR("Invalid repeater mode: %d", mode);
        return -EINVAL;
    }
    
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CONFIG_REG, config_val);
    if (ret) {
        LOG_ERR("Failed to set repeater mode: %d", ret);
        return ret;
    }
    
    /* Wait for mode change to take effect */
    k_msleep(5);
    
    return 0;
}

/**
 * @brief Set signal detection threshold
 */
int qcom_repeater_set_signal_threshold(struct qcom_phy_data *phy, uint8_t threshold)
{
    int ret;
    
    if (!phy || !phy->repeater_dev) {
        LOG_ERR("Invalid repeater device");
        return -EINVAL;
    }
    
    if (threshold > QCOM_REPEATER_SIGNAL_DETECT_HIGH) {
        LOG_ERR("Invalid signal detection threshold: %d", threshold);
        return -EINVAL;
    }
    
    LOG_INF("Setting repeater signal detection threshold: %d", threshold);
    
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_SIGNAL_DETECT_REG, threshold);
    if (ret) {
        LOG_ERR("Failed to set repeater signal threshold: %d", ret);
        return ret;
    }
    
    /* Wait for setting to take effect */
    k_msleep(2);
    
    return 0;
}

/**
 * @brief Enable/disable CDR (Clock Data Recovery)
 */
int qcom_repeater_set_cdr(struct qcom_phy_data *phy, bool enable)
{
    int ret;
    uint8_t cdr_val;
    
    if (!phy || !phy->repeater_dev) {
        LOG_ERR("Invalid repeater device");
        return -EINVAL;
    }
    
    cdr_val = enable ? QCOM_REPEATER_CDR_ENABLE : QCOM_REPEATER_CDR_DISABLE;
    
    LOG_INF("%s repeater CDR", enable ? "Enabling" : "Disabling");
    
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CDR_CONTROL_REG, cdr_val);
    if (ret) {
        LOG_ERR("Failed to %s repeater CDR: %d", enable ? "enable" : "disable", ret);
        return ret;
    }
    
    /* Wait for CDR state change */
    k_msleep(enable ? 5 : 2);
    
    return 0;
}

/**
 * @brief Get repeater status
 */
int qcom_repeater_get_status(struct qcom_phy_data *phy, uint8_t *status)
{
    int ret;
    
    if (!phy || !phy->repeater_dev || !status) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    ret = i2c_reg_read_byte(phy->repeater_dev, phy->repeater_addr,
                           QCOM_REPEATER_STATUS_REG, status);
    if (ret) {
        LOG_ERR("Failed to read repeater status: %d", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Check for loss of signal
 */
int qcom_repeater_check_signal_loss(struct qcom_phy_data *phy, bool *signal_lost)
{
    int ret;
    uint8_t los_status;
    
    if (!phy || !phy->repeater_dev || !signal_lost) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    ret = i2c_reg_read_byte(phy->repeater_dev, phy->repeater_addr,
                           QCOM_REPEATER_LOSS_OF_SIGNAL_REG, &los_status);
    if (ret) {
        LOG_ERR("Failed to read repeater loss of signal status: %d", ret);
        return ret;
    }
    
    *signal_lost = (los_status & 0x01) != 0;
    
    if (*signal_lost) {
        LOG_WRN("Repeater detected loss of signal");
    }
    
    return 0;
}

/**
 * @brief Configure repeater for USB3 operation
 */
int qcom_repeater_configure_usb3(struct qcom_phy_data *phy, usb_speed_t speed)
{
    int ret;
    uint8_t threshold;
    
    if (!phy || !phy->repeater_dev) {
        LOG_ERR("Invalid repeater device");
        return -EINVAL;
    }
    
    /* Configure repeater settings based on USB3 speed */
    switch (speed) {
    case USB_SPEED_SUPER:
        /* USB3.0 SuperSpeed - standard settings */
        threshold = QCOM_REPEATER_SIGNAL_DETECT_MED;
        LOG_INF("Configuring repeater for USB3.0 SuperSpeed");
        break;
        
    case USB_SPEED_SUPER_PLUS:
        /* USB3.1 SuperSpeed+ - higher sensitivity */
        threshold = QCOM_REPEATER_SIGNAL_DETECT_HIGH;
        LOG_INF("Configuring repeater for USB3.1 SuperSpeed+");
        break;
        
    default:
        LOG_WRN("Unsupported USB3 speed: %d", speed);
        threshold = QCOM_REPEATER_SIGNAL_DETECT_MED;
        break;
    }
    
    /* Set repeater to USB3 mode */
    ret = qcom_repeater_set_mode(phy, QCOM_REPEATER_MODE_USB3);
    if (ret) {
        LOG_ERR("Failed to set repeater to USB3 mode: %d", ret);
        return ret;
    }
    
    /* Set appropriate signal detection threshold */
    ret = qcom_repeater_set_signal_threshold(phy, threshold);
    if (ret) {
        LOG_ERR("Failed to set repeater signal threshold for USB3: %d", ret);
        return ret;
    }
    
    /* Enable CDR for USB3 */
    ret = qcom_repeater_set_cdr(phy, true);
    if (ret) {
        LOG_ERR("Failed to enable repeater CDR for USB3: %d", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Enable/disable repeater
 */
int qcom_repeater_enable(struct qcom_phy_data *phy, bool enable)
{
    int ret;
    uint8_t control_val;
    
    if (!phy || !phy->repeater_dev) {
        LOG_ERR("Invalid repeater device");
        return -EINVAL;
    }
    
    control_val = enable ? QCOM_REPEATER_ENABLE : QCOM_REPEATER_DISABLE;
    
    LOG_INF("%s repeater", enable ? "Enabling" : "Disabling");
    
    ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                            QCOM_REPEATER_CONTROL_REG, control_val);
    if (ret) {
        LOG_ERR("Failed to %s repeater: %d", enable ? "enable" : "disable", ret);
        return ret;
    }
    
    /* Wait for state change */
    k_msleep(enable ? 5 : 2);
    
    return 0;
}

/**
 * @brief Reset repeater
 */
int qcom_repeater_reset(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy) {
        LOG_ERR("Invalid PHY data");
        return -EINVAL;
    }
    
    LOG_INF("Resetting repeater");
    
    /* Assert reset if GPIO is available */
    if (phy->repeater_reset_gpio.port) {
        ret = gpio_pin_set_dt(&phy->repeater_reset_gpio, 1);
        if (ret) {
            LOG_ERR("Failed to assert repeater reset: %d", ret);
            return ret;
        }
        
        k_msleep(1);
        
        ret = gpio_pin_set_dt(&phy->repeater_reset_gpio, 0);
        if (ret) {
            LOG_ERR("Failed to deassert repeater reset: %d", ret);
            return ret;
        }
        
        /* Wait for reset to complete */
        k_msleep(10);
    } else if (phy->repeater_dev) {
        /* Software reset via I2C if no GPIO reset */
        ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                                QCOM_REPEATER_CONTROL_REG, QCOM_REPEATER_RESET);
        if (ret) {
            LOG_ERR("Failed to reset repeater via I2C: %d", ret);
            return ret;
        }
        
        /* Wait for reset to complete */
        k_msleep(10);
        
        /* Re-enable repeater after reset */
        ret = i2c_reg_write_byte(phy->repeater_dev, phy->repeater_addr,
                                QCOM_REPEATER_CONTROL_REG, QCOM_REPEATER_ENABLE);
        if (ret) {
            LOG_ERR("Failed to re-enable repeater after reset: %d", ret);
            return ret;
        }
    }
    
    return 0;
}

/**
 * @brief Monitor repeater signal quality
 */
int qcom_repeater_monitor_signal(struct qcom_phy_data *phy, 
                                struct qcom_repeater_signal_info *signal_info)
{
    int ret;
    uint8_t status, los_status;
    bool signal_lost;
    
    if (!phy || !phy->repeater_dev || !signal_info) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    /* Get repeater status */
    ret = qcom_repeater_get_status(phy, &status);
    if (ret) {
        LOG_ERR("Failed to get repeater status: %d", ret);
        return ret;
    }
    
    /* Check for signal loss */
    ret = qcom_repeater_check_signal_loss(phy, &signal_lost);
    if (ret) {
        LOG_ERR("Failed to check repeater signal loss: %d", ret);
        return ret;
    }
    
    /* Fill signal info structure */
    signal_info->status = status;
    signal_info->signal_detected = (status & 0x01) != 0;
    signal_info->cdr_locked = (status & 0x02) != 0;
    signal_info->signal_lost = signal_lost;
    signal_info->current_mode = (status >> 4) & 0x03;
    
    LOG_DBG("Repeater signal info: detected=%d, cdr_locked=%d, lost=%d, mode=%d",
            signal_info->signal_detected, signal_info->cdr_locked,
            signal_info->signal_lost, signal_info->current_mode);
    
    return 0;
}

/**
 * @brief Deinitialize repeater
 */
int qcom_repeater_deinit(struct qcom_phy_data *phy)
{
    if (!phy) {
        LOG_ERR("Invalid PHY data");
        return -EINVAL;
    }
    
    LOG_INF("Deinitializing repeater");
    
    /* Disable repeater */
    qcom_repeater_enable(phy, false);
    
    return 0;
}
