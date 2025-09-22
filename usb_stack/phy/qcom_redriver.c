/*
 * Qualcomm USB Redriver Support
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
#include "qcom_phy_internal.h"

LOG_MODULE_REGISTER(qcom_redriver, CONFIG_USB_STACK_LOG_LEVEL);

/* Qualcomm USB Redriver Register Definitions */
#define QCOM_REDRIVER_CHIP_ID_REG           0x00
#define QCOM_REDRIVER_CHIP_REV_REG          0x01
#define QCOM_REDRIVER_GENERAL_CFG_REG       0x02
#define QCOM_REDRIVER_EQ_CFG_REG            0x03
#define QCOM_REDRIVER_OUTPUT_CFG_REG        0x04
#define QCOM_REDRIVER_CHANNEL_CFG_REG       0x05
#define QCOM_REDRIVER_AUX_CFG_REG           0x06
#define QCOM_REDRIVER_STATUS_REG            0x07

/* Redriver Configuration Values */
#define QCOM_REDRIVER_ENABLE                0x01
#define QCOM_REDRIVER_DISABLE               0x00

/* Equalization Settings */
#define QCOM_REDRIVER_EQ_LEVEL_0            0x00
#define QCOM_REDRIVER_EQ_LEVEL_1            0x01
#define QCOM_REDRIVER_EQ_LEVEL_2            0x02
#define QCOM_REDRIVER_EQ_LEVEL_3            0x03

/* Output Swing Settings */
#define QCOM_REDRIVER_SWING_800MV           0x00
#define QCOM_REDRIVER_SWING_900MV           0x01
#define QCOM_REDRIVER_SWING_1000MV          0x02
#define QCOM_REDRIVER_SWING_1100MV          0x03

/* Channel Configuration */
#define QCOM_REDRIVER_CHANNEL_USB3          0x01
#define QCOM_REDRIVER_CHANNEL_DP             0x02
#define QCOM_REDRIVER_CHANNEL_USB3_DP        0x03

/**
 * @brief Initialize Qualcomm USB redriver
 */
int qcom_redriver_init(struct qcom_phy_data *phy)
{
    int ret;
    uint8_t chip_id, chip_rev;
    
    if (!phy || !phy->redriver_dev) {
        LOG_ERR("Invalid redriver device");
        return -EINVAL;
    }
    
    LOG_INF("Initializing Qualcomm USB redriver");
    
    /* Read chip ID to verify redriver presence */
    ret = i2c_reg_read_byte(phy->redriver_dev, phy->redriver_addr,
                           QCOM_REDRIVER_CHIP_ID_REG, &chip_id);
    if (ret) {
        LOG_ERR("Failed to read redriver chip ID: %d", ret);
        return ret;
    }
    
    ret = i2c_reg_read_byte(phy->redriver_dev, phy->redriver_addr,
                           QCOM_REDRIVER_CHIP_REV_REG, &chip_rev);
    if (ret) {
        LOG_ERR("Failed to read redriver chip revision: %d", ret);
        return ret;
    }
    
    LOG_INF("Redriver detected: ID=0x%02x, Revision=0x%02x", chip_id, chip_rev);
    
    /* Configure general settings */
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_GENERAL_CFG_REG, QCOM_REDRIVER_ENABLE);
    if (ret) {
        LOG_ERR("Failed to enable redriver: %d", ret);
        return ret;
    }
    
    /* Set default equalization level */
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_EQ_CFG_REG, QCOM_REDRIVER_EQ_LEVEL_2);
    if (ret) {
        LOG_ERR("Failed to set redriver equalization: %d", ret);
        return ret;
    }
    
    /* Set default output swing */
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_OUTPUT_CFG_REG, QCOM_REDRIVER_SWING_1000MV);
    if (ret) {
        LOG_ERR("Failed to set redriver output swing: %d", ret);
        return ret;
    }
    
    /* Configure for USB3 + DP channels */
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_CHANNEL_CFG_REG, QCOM_REDRIVER_CHANNEL_USB3_DP);
    if (ret) {
        LOG_ERR("Failed to configure redriver channels: %d", ret);
        return ret;
    }
    
    /* Wait for redriver to stabilize */
    k_msleep(5);
    
    LOG_DBG("Qualcomm USB redriver initialized successfully");
    return 0;
}

/**
 * @brief Configure redriver equalization
 */
int qcom_redriver_set_equalization(struct qcom_phy_data *phy, uint8_t eq_level)
{
    int ret;
    
    if (!phy || !phy->redriver_dev) {
        LOG_ERR("Invalid redriver device");
        return -EINVAL;
    }
    
    if (eq_level > QCOM_REDRIVER_EQ_LEVEL_3) {
        LOG_ERR("Invalid equalization level: %d", eq_level);
        return -EINVAL;
    }
    
    LOG_INF("Setting redriver equalization level: %d", eq_level);
    
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_EQ_CFG_REG, eq_level);
    if (ret) {
        LOG_ERR("Failed to set redriver equalization: %d", ret);
        return ret;
    }
    
    /* Wait for setting to take effect */
    k_msleep(2);
    
    return 0;
}

/**
 * @brief Configure redriver output swing
 */
int qcom_redriver_set_output_swing(struct qcom_phy_data *phy, uint8_t swing_level)
{
    int ret;
    
    if (!phy || !phy->redriver_dev) {
        LOG_ERR("Invalid redriver device");
        return -EINVAL;
    }
    
    if (swing_level > QCOM_REDRIVER_SWING_1100MV) {
        LOG_ERR("Invalid output swing level: %d", swing_level);
        return -EINVAL;
    }
    
    LOG_INF("Setting redriver output swing level: %d", swing_level);
    
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_OUTPUT_CFG_REG, swing_level);
    if (ret) {
        LOG_ERR("Failed to set redriver output swing: %d", ret);
        return ret;
    }
    
    /* Wait for setting to take effect */
    k_msleep(2);
    
    return 0;
}

/**
 * @brief Configure redriver for specific channel mode
 */
int qcom_redriver_set_channel_mode(struct qcom_phy_data *phy, qcom_redriver_channel_t mode)
{
    int ret;
    uint8_t channel_config;
    
    if (!phy || !phy->redriver_dev) {
        LOG_ERR("Invalid redriver device");
        return -EINVAL;
    }
    
    switch (mode) {
    case QCOM_REDRIVER_CHANNEL_USB3_ONLY:
        channel_config = QCOM_REDRIVER_CHANNEL_USB3;
        LOG_INF("Setting redriver to USB3 only mode");
        break;
        
    case QCOM_REDRIVER_CHANNEL_DP_ONLY:
        channel_config = QCOM_REDRIVER_CHANNEL_DP;
        LOG_INF("Setting redriver to DisplayPort only mode");
        break;
        
    case QCOM_REDRIVER_CHANNEL_USB3_DP_COMBO:
        channel_config = QCOM_REDRIVER_CHANNEL_USB3_DP;
        LOG_INF("Setting redriver to USB3 + DisplayPort combo mode");
        break;
        
    default:
        LOG_ERR("Invalid redriver channel mode: %d", mode);
        return -EINVAL;
    }
    
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_CHANNEL_CFG_REG, channel_config);
    if (ret) {
        LOG_ERR("Failed to set redriver channel mode: %d", ret);
        return ret;
    }
    
    /* Wait for mode change to take effect */
    k_msleep(5);
    
    return 0;
}

/**
 * @brief Get redriver status
 */
int qcom_redriver_get_status(struct qcom_phy_data *phy, uint8_t *status)
{
    int ret;
    
    if (!phy || !phy->redriver_dev || !status) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    ret = i2c_reg_read_byte(phy->redriver_dev, phy->redriver_addr,
                           QCOM_REDRIVER_STATUS_REG, status);
    if (ret) {
        LOG_ERR("Failed to read redriver status: %d", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Configure redriver for USB3 SuperSpeed operation
 */
int qcom_redriver_configure_usb3(struct qcom_phy_data *phy, usb_speed_t speed)
{
    int ret;
    uint8_t eq_level, swing_level;
    
    if (!phy || !phy->redriver_dev) {
        LOG_ERR("Invalid redriver device");
        return -EINVAL;
    }
    
    /* Configure redriver settings based on USB3 speed */
    switch (speed) {
    case USB_SPEED_SUPER:
        /* USB3.0 SuperSpeed - moderate settings */
        eq_level = QCOM_REDRIVER_EQ_LEVEL_2;
        swing_level = QCOM_REDRIVER_SWING_1000MV;
        LOG_INF("Configuring redriver for USB3.0 SuperSpeed");
        break;
        
    case USB_SPEED_SUPER_PLUS:
        /* USB3.1 SuperSpeed+ - higher performance settings */
        eq_level = QCOM_REDRIVER_EQ_LEVEL_3;
        swing_level = QCOM_REDRIVER_SWING_1100MV;
        LOG_INF("Configuring redriver for USB3.1 SuperSpeed+");
        break;
        
    default:
        LOG_WRN("Unsupported USB3 speed: %d", speed);
        eq_level = QCOM_REDRIVER_EQ_LEVEL_2;
        swing_level = QCOM_REDRIVER_SWING_1000MV;
        break;
    }
    
    /* Set equalization level */
    ret = qcom_redriver_set_equalization(phy, eq_level);
    if (ret) {
        LOG_ERR("Failed to set redriver equalization for USB3: %d", ret);
        return ret;
    }
    
    /* Set output swing */
    ret = qcom_redriver_set_output_swing(phy, swing_level);
    if (ret) {
        LOG_ERR("Failed to set redriver output swing for USB3: %d", ret);
        return ret;
    }
    
    /* Configure for USB3 channel */
    ret = qcom_redriver_set_channel_mode(phy, QCOM_REDRIVER_CHANNEL_USB3_ONLY);
    if (ret) {
        LOG_ERR("Failed to set redriver channel mode for USB3: %d", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Enable/disable redriver
 */
int qcom_redriver_enable(struct qcom_phy_data *phy, bool enable)
{
    int ret;
    uint8_t config_val;
    
    if (!phy || !phy->redriver_dev) {
        LOG_ERR("Invalid redriver device");
        return -EINVAL;
    }
    
    config_val = enable ? QCOM_REDRIVER_ENABLE : QCOM_REDRIVER_DISABLE;
    
    LOG_INF("%s redriver", enable ? "Enabling" : "Disabling");
    
    ret = i2c_reg_write_byte(phy->redriver_dev, phy->redriver_addr,
                            QCOM_REDRIVER_GENERAL_CFG_REG, config_val);
    if (ret) {
        LOG_ERR("Failed to %s redriver: %d", enable ? "enable" : "disable", ret);
        return ret;
    }
    
    /* Wait for state change */
    k_msleep(enable ? 5 : 2);
    
    return 0;
}

/**
 * @brief Reset redriver
 */
int qcom_redriver_reset(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy) {
        LOG_ERR("Invalid PHY data");
        return -EINVAL;
    }
    
    LOG_INF("Resetting redriver");
    
    /* Assert reset if GPIO is available */
    if (phy->redriver_reset_gpio.port) {
        ret = gpio_pin_set_dt(&phy->redriver_reset_gpio, 1);
        if (ret) {
            LOG_ERR("Failed to assert redriver reset: %d", ret);
            return ret;
        }
        
        k_msleep(1);
        
        ret = gpio_pin_set_dt(&phy->redriver_reset_gpio, 0);
        if (ret) {
            LOG_ERR("Failed to deassert redriver reset: %d", ret);
            return ret;
        }
        
        /* Wait for reset to complete */
        k_msleep(10);
    } else {
        /* Software reset via I2C if no GPIO reset */
        ret = qcom_redriver_enable(phy, false);
        if (ret) {
            LOG_ERR("Failed to disable redriver for reset: %d", ret);
            return ret;
        }
        
        k_msleep(5);
        
        ret = qcom_redriver_enable(phy, true);
        if (ret) {
            LOG_ERR("Failed to re-enable redriver after reset: %d", ret);
            return ret;
        }
    }
    
    return 0;
}

/**
 * @brief Deinitialize redriver
 */
int qcom_redriver_deinit(struct qcom_phy_data *phy)
{
    if (!phy) {
        LOG_ERR("Invalid PHY data");
        return -EINVAL;
    }
    
    LOG_INF("Deinitializing redriver");
    
    /* Disable redriver */
    qcom_redriver_enable(phy, false);
    
    return 0;
}
