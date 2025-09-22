/*
 * Qualcomm USB PHY Internal Definitions
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QCOM_PHY_INTERNAL_H
#define QCOM_PHY_INTERNAL_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "../include/usb_stack_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct qcom_phy_tune_param;

/* Redriver Channel Modes */
typedef enum {
    QCOM_REDRIVER_CHANNEL_USB3_ONLY = 0,
    QCOM_REDRIVER_CHANNEL_DP_ONLY,
    QCOM_REDRIVER_CHANNEL_USB3_DP_COMBO
} qcom_redriver_channel_t;

/* Repeater Modes */
typedef enum {
    QCOM_REPEATER_MODE_USB2 = 0,
    QCOM_REPEATER_MODE_USB3,
    QCOM_REPEATER_MODE_AUTO
} qcom_repeater_mode_t;

/* Repeater Signal Information */
struct qcom_repeater_signal_info {
    uint8_t status;
    bool signal_detected;
    bool cdr_locked;
    bool signal_lost;
    uint8_t current_mode;
};

/* USB Speed enumeration for redriver configuration */
typedef enum {
    USB_SPEED_UNKNOWN = 0,
    USB_SPEED_LOW,
    USB_SPEED_FULL,
    USB_SPEED_HIGH,
    USB_SPEED_SUPER,
    USB_SPEED_SUPER_PLUS
} usb_speed_t;

/* PHY Tuning Parameter Structure */
struct qcom_phy_tune_param {
    uint32_t offset;
    uint32_t value;
};

/* PHY Private Data Structure */
struct qcom_phy_data {
    /* Base addresses */
    uintptr_t qusb2_base;
    uintptr_t qmp_base;
    uintptr_t pcs_base;
    
    /* PHY type */
    usb_stack_phy_type_t type;
    
    /* Power state */
    bool powered;
    bool initialized;
    
    /* Clocks and resets */
    const struct device *clock_dev;
    const struct device *reset_dev;
    uint32_t clock_ids[8];
    uint32_t reset_ids[4];
    uint8_t num_clocks;
    uint8_t num_resets;
    
    /* Tuning parameters */
    struct qcom_phy_tune_param *tune_params;
    uint32_t num_tune_params;
    
    /* Redriver support */
    const struct device *redriver_dev;
    uint16_t redriver_addr;
    struct gpio_dt_spec redriver_reset_gpio;
    struct gpio_dt_spec redriver_enable_gpio;
    
    /* Retimer support */
    const struct device *retimer_dev;
    uint16_t retimer_addr;
    struct gpio_dt_spec retimer_reset_gpio;
    struct gpio_dt_spec retimer_enable_gpio;
    
    /* Repeater support */
    const struct device *repeater_dev;
    uint16_t repeater_addr;
    struct gpio_dt_spec repeater_reset_gpio;
    struct gpio_dt_spec repeater_enable_gpio;
};

/* PHY Configuration Structure */
struct qcom_phy_config {
    uintptr_t qusb2_base;
    uintptr_t qmp_base;
    uintptr_t pcs_base;
    usb_stack_phy_type_t type;
    const struct device *clock_dev;
    const struct device *reset_dev;
    uint32_t *clock_ids;
    uint32_t *reset_ids;
    uint8_t num_clocks;
    uint8_t num_resets;
    struct qcom_phy_tune_param *tune_params;
    uint32_t num_tune_params;
    
    /* Redriver configuration */
    const struct device *redriver_dev;
    uint16_t redriver_addr;
    struct gpio_dt_spec redriver_reset_gpio;
    struct gpio_dt_spec redriver_enable_gpio;
    
    /* Retimer configuration */
    const struct device *retimer_dev;
    uint16_t retimer_addr;
    struct gpio_dt_spec retimer_reset_gpio;
    struct gpio_dt_spec retimer_enable_gpio;
    
    /* Repeater configuration */
    const struct device *repeater_dev;
    uint16_t repeater_addr;
    struct gpio_dt_spec repeater_reset_gpio;
    struct gpio_dt_spec repeater_enable_gpio;
};

/* Function declarations for redriver */
int qcom_redriver_init(struct qcom_phy_data *phy);
int qcom_redriver_deinit(struct qcom_phy_data *phy);
int qcom_redriver_enable(struct qcom_phy_data *phy, bool enable);
int qcom_redriver_reset(struct qcom_phy_data *phy);
int qcom_redriver_set_equalization(struct qcom_phy_data *phy, uint8_t eq_level);
int qcom_redriver_set_output_swing(struct qcom_phy_data *phy, uint8_t swing_level);
int qcom_redriver_set_channel_mode(struct qcom_phy_data *phy, qcom_redriver_channel_t mode);
int qcom_redriver_configure_usb3(struct qcom_phy_data *phy, usb_speed_t speed);
int qcom_redriver_get_status(struct qcom_phy_data *phy, uint8_t *status);

/* Function declarations for retimer */
int qcom_retimer_init(struct qcom_phy_data *phy);
int qcom_retimer_deinit(struct qcom_phy_data *phy);
int qcom_retimer_enable(struct qcom_phy_data *phy, bool enable);
int qcom_retimer_reset(struct qcom_phy_data *phy);
int qcom_retimer_set_orientation(struct qcom_phy_data *phy, usb_stack_typec_orientation_t orientation);
int qcom_retimer_configure_usb3(struct qcom_phy_data *phy, usb_speed_t speed);
int qcom_retimer_configure_dp(struct qcom_phy_data *phy, uint8_t lanes);
int qcom_retimer_get_status(struct qcom_phy_data *phy, uint8_t *status);

/* Function declarations for repeater */
int qcom_repeater_init(struct qcom_phy_data *phy);
int qcom_repeater_deinit(struct qcom_phy_data *phy);
int qcom_repeater_enable(struct qcom_phy_data *phy, bool enable);
int qcom_repeater_reset(struct qcom_phy_data *phy);
int qcom_repeater_set_mode(struct qcom_phy_data *phy, qcom_repeater_mode_t mode);
int qcom_repeater_set_signal_threshold(struct qcom_phy_data *phy, uint8_t threshold);
int qcom_repeater_set_cdr(struct qcom_phy_data *phy, bool enable);
int qcom_repeater_get_status(struct qcom_phy_data *phy, uint8_t *status);
int qcom_repeater_check_signal_loss(struct qcom_phy_data *phy, bool *signal_lost);
int qcom_repeater_configure_usb3(struct qcom_phy_data *phy, usb_speed_t speed);
int qcom_repeater_monitor_signal(struct qcom_phy_data *phy, struct qcom_repeater_signal_info *signal_info);

#ifdef __cplusplus
}
#endif

#endif /* QCOM_PHY_INTERNAL_H */
