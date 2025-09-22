/*
 * Qualcomm USB PHY Driver for Zephyr USB Stack
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

#include "../include/usb_stack.h"
#include "qcom_phy_regs.h"
#include "qcom_phy_internal.h"

LOG_MODULE_REGISTER(qcom_phy, CONFIG_USB_STACK_LOG_LEVEL);

/* QUSB2 PHY Register Offsets */
#define QUSB2PHY_PWR_CTRL1              0x210
#define QUSB2PHY_PWR_CTRL2              0x214
#define QUSB2PHY_IMP_CTRL1              0x220
#define QUSB2PHY_IMP_CTRL2              0x224
#define QUSB2PHY_CHG_CTRL2              0x23C
#define QUSB2PHY_TUNE1                  0x240
#define QUSB2PHY_TUNE2                  0x244
#define QUSB2PHY_TUNE3                  0x248
#define QUSB2PHY_TUNE4                  0x24C
#define QUSB2PHY_TUNE5                  0x250
#define QUSB2PHY_DIG_CTRL1              0x254
#define QUSB2PHY_DIG_CTRL2              0x258

/* QMP USB3 PHY Register Offsets */
#define QSERDES_COM_BG_TIMER            0x00C
#define QSERDES_COM_SSC_EN_CENTER       0x010
#define QSERDES_COM_SSC_ADJ_PER1        0x014
#define QSERDES_COM_SSC_PER1            0x01C
#define QSERDES_COM_SSC_PER2            0x020
#define QSERDES_COM_SSC_STEP_SIZE1      0x024
#define QSERDES_COM_SSC_STEP_SIZE2      0x028
#define QSERDES_COM_BIAS_EN_CLKBUFLR_EN 0x034
#define QSERDES_COM_CLK_ENABLE1         0x038
#define QSERDES_COM_SYS_CLK_CTRL        0x03C
#define QSERDES_COM_SYSCLK_BUF_ENABLE   0x040
#define QSERDES_COM_PLL_IVCO            0x048
#define QSERDES_COM_LOCK_CMP1_MODE0     0x04C
#define QSERDES_COM_LOCK_CMP2_MODE0     0x050
#define QSERDES_COM_LOCK_CMP3_MODE0     0x054
#define QSERDES_COM_LOCK_CMP1_MODE1     0x058
#define QSERDES_COM_LOCK_CMP2_MODE1     0x05C
#define QSERDES_COM_LOCK_CMP3_MODE1     0x060
#define QSERDES_COM_CP_CTRL_MODE0       0x078
#define QSERDES_COM_CP_CTRL_MODE1       0x07C
#define QSERDES_COM_PLL_RCTRL_MODE0     0x084
#define QSERDES_COM_PLL_RCTRL_MODE1     0x088
#define QSERDES_COM_PLL_CCTRL_MODE0     0x090
#define QSERDES_COM_PLL_CCTRL_MODE1     0x094
#define QSERDES_COM_SYSCLK_EN_SEL       0x0AC
#define QSERDES_COM_RESETSM_CNTRL       0x0B4
#define QSERDES_COM_RESTRIM_CTRL        0x0BC
#define QSERDES_COM_RESCODE_DIV_NUM     0x0C4
#define QSERDES_COM_LOCK_CMP_EN         0x0C8
#define QSERDES_COM_LOCK_CMP_CFG        0x0CC
#define QSERDES_COM_DEC_START_MODE0     0x0D0
#define QSERDES_COM_DEC_START_MODE1     0x0D4
#define QSERDES_COM_DIV_FRAC_START1_MODE0 0x0DC
#define QSERDES_COM_DIV_FRAC_START2_MODE0 0x0E0
#define QSERDES_COM_DIV_FRAC_START3_MODE0 0x0E4
#define QSERDES_COM_DIV_FRAC_START1_MODE1 0x0E8
#define QSERDES_COM_DIV_FRAC_START2_MODE1 0x0EC
#define QSERDES_COM_DIV_FRAC_START3_MODE1 0x0F0
#define QSERDES_COM_INTEGLOOP_GAIN0_MODE0 0x108
#define QSERDES_COM_INTEGLOOP_GAIN1_MODE0 0x10C
#define QSERDES_COM_INTEGLOOP_GAIN0_MODE1 0x110
#define QSERDES_COM_INTEGLOOP_GAIN1_MODE1 0x114
#define QSERDES_COM_VCO_TUNE_CTRL       0x124
#define QSERDES_COM_VCO_TUNE_MAP        0x128
#define QSERDES_COM_VCO_TUNE1_MODE0     0x12C
#define QSERDES_COM_VCO_TUNE2_MODE0     0x130
#define QSERDES_COM_VCO_TUNE1_MODE1     0x134
#define QSERDES_COM_VCO_TUNE2_MODE1     0x138
#define QSERDES_COM_VCO_TUNE_TIMER1     0x144
#define QSERDES_COM_VCO_TUNE_TIMER2     0x148
#define QSERDES_COM_SAR                 0x188
#define QSERDES_COM_SAR_CLK             0x18C
#define QSERDES_COM_SAR_CODE_OUT_STATUS 0x190
#define QSERDES_COM_SAR_CODE_READY_STATUS 0x194
#define QSERDES_COM_CMN_STATUS          0x1C0


/* Register access helpers */
static inline uint32_t qcom_phy_readl(uintptr_t base, uint32_t offset)
{
    return sys_read32(base + offset);
}

static inline void qcom_phy_writel(uintptr_t base, uint32_t offset, uint32_t value)
{
    sys_write32(value, base + offset);
}

/* QUSB2 PHY initialization sequence */
static int qusb2_phy_init_sequence(struct qcom_phy_data *phy)
{
    uintptr_t base = phy->qusb2_base;
    
    LOG_DBG("Initializing QUSB2 PHY");
    
    /* Power up the PHY */
    qcom_phy_writel(base, QUSB2PHY_PWR_CTRL1, 0x00);
    
    /* Configure impedance control */
    qcom_phy_writel(base, QUSB2PHY_IMP_CTRL1, 0x08);
    qcom_phy_writel(base, QUSB2PHY_IMP_CTRL2, 0x58);
    
    /* Configure charge control */
    qcom_phy_writel(base, QUSB2PHY_CHG_CTRL2, 0x22);
    
    /* Apply tuning parameters */
    qcom_phy_writel(base, QUSB2PHY_TUNE1, 0xF8);
    qcom_phy_writel(base, QUSB2PHY_TUNE2, 0xB3);
    qcom_phy_writel(base, QUSB2PHY_TUNE3, 0x83);
    qcom_phy_writel(base, QUSB2PHY_TUNE4, 0xC0);
    qcom_phy_writel(base, QUSB2PHY_TUNE5, 0x00);
    
    /* Configure digital control */
    qcom_phy_writel(base, QUSB2PHY_DIG_CTRL1, 0x80);
    qcom_phy_writel(base, QUSB2PHY_DIG_CTRL2, 0x15);
    
    /* Apply custom tuning parameters if available */
    for (uint32_t i = 0; i < phy->num_tune_params; i++) {
        qcom_phy_writel(base, phy->tune_params[i].offset, phy->tune_params[i].value);
    }
    
    /* Wait for PHY to stabilize */
    k_msleep(1);
    
    return 0;
}

/* QMP USB3 PHY initialization sequence */
static int qmp_usb3_phy_init_sequence(struct qcom_phy_data *phy)
{
    uintptr_t serdes_base = phy->qmp_base;
    uintptr_t pcs_base = phy->pcs_base;
    uint32_t val;
    int timeout = 1000;
    
    LOG_DBG("Initializing QMP USB3 PHY");
    
    /* Configure SERDES common block */
    qcom_phy_writel(serdes_base, QSERDES_COM_BG_TIMER, 0x09);
    qcom_phy_writel(serdes_base, QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x08);
    qcom_phy_writel(serdes_base, QSERDES_COM_CLK_ENABLE1, 0x90);
    qcom_phy_writel(serdes_base, QSERDES_COM_SYS_CLK_CTRL, 0x02);
    qcom_phy_writel(serdes_base, QSERDES_COM_SYSCLK_BUF_ENABLE, 0x0A);
    qcom_phy_writel(serdes_base, QSERDES_COM_SYSCLK_EN_SEL, 0x04);
    qcom_phy_writel(serdes_base, QSERDES_COM_CP_CTRL_MODE0, 0x06);
    qcom_phy_writel(serdes_base, QSERDES_COM_CP_CTRL_MODE1, 0x06);
    qcom_phy_writel(serdes_base, QSERDES_COM_PLL_RCTRL_MODE0, 0x16);
    qcom_phy_writel(serdes_base, QSERDES_COM_PLL_RCTRL_MODE1, 0x16);
    qcom_phy_writel(serdes_base, QSERDES_COM_PLL_CCTRL_MODE0, 0x36);
    qcom_phy_writel(serdes_base, QSERDES_COM_PLL_CCTRL_MODE1, 0x36);
    qcom_phy_writel(serdes_base, QSERDES_COM_PLL_IVCO, 0x0F);
    
    /* Configure lock comparators */
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP1_MODE0, 0x04);
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP2_MODE0, 0x28);
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP3_MODE0, 0x00);
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP1_MODE1, 0x06);
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP2_MODE1, 0x2E);
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP3_MODE1, 0x00);
    
    /* Configure VCO settings */
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE_CTRL, 0x00);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE_MAP, 0x00);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE1_MODE0, 0xB4);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE2_MODE0, 0x02);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE1_MODE1, 0xB4);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE2_MODE1, 0x02);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE_TIMER1, 0xFF);
    qcom_phy_writel(serdes_base, QSERDES_COM_VCO_TUNE_TIMER2, 0x3F);
    
    /* Configure decimation and fractional settings */
    qcom_phy_writel(serdes_base, QSERDES_COM_DEC_START_MODE0, 0x82);
    qcom_phy_writel(serdes_base, QSERDES_COM_DEC_START_MODE1, 0x82);
    qcom_phy_writel(serdes_base, QSERDES_COM_DIV_FRAC_START1_MODE0, 0x55);
    qcom_phy_writel(serdes_base, QSERDES_COM_DIV_FRAC_START2_MODE0, 0x55);
    qcom_phy_writel(serdes_base, QSERDES_COM_DIV_FRAC_START3_MODE0, 0x03);
    qcom_phy_writel(serdes_base, QSERDES_COM_DIV_FRAC_START1_MODE1, 0x55);
    qcom_phy_writel(serdes_base, QSERDES_COM_DIV_FRAC_START2_MODE1, 0x55);
    qcom_phy_writel(serdes_base, QSERDES_COM_DIV_FRAC_START3_MODE1, 0x03);
    
    /* Configure integration loop gain */
    qcom_phy_writel(serdes_base, QSERDES_COM_INTEGLOOP_GAIN0_MODE0, 0x80);
    qcom_phy_writel(serdes_base, QSERDES_COM_INTEGLOOP_GAIN1_MODE0, 0x00);
    qcom_phy_writel(serdes_base, QSERDES_COM_INTEGLOOP_GAIN0_MODE1, 0x80);
    qcom_phy_writel(serdes_base, QSERDES_COM_INTEGLOOP_GAIN1_MODE1, 0x00);
    
    /* Configure lock detection */
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP_EN, 0x01);
    qcom_phy_writel(serdes_base, QSERDES_COM_LOCK_CMP_CFG, 0x90);
    
    /* Configure reset state machine */
    qcom_phy_writel(serdes_base, QSERDES_COM_RESETSM_CNTRL, 0x20);
    
    /* Start PLL */
    qcom_phy_writel(pcs_base, QPHY_POWER_DOWN_CONTROL, 0x01);
    qcom_phy_writel(pcs_base, QPHY_START_CTRL, 0x00);
    qcom_phy_writel(pcs_base, QPHY_START_CTRL, 0x03);
    
    /* Wait for PLL lock */
    do {
        val = qcom_phy_readl(pcs_base, QPHY_PCS_STATUS);
        if (val & PHYSTATUS) {
            break;
        }
        k_usleep(1);
    } while (--timeout);
    
    if (!timeout) {
        LOG_ERR("QMP USB3 PHY PLL lock timeout");
        return -ETIMEDOUT;
    }
    
    LOG_DBG("QMP USB3 PHY PLL locked successfully");
    return 0;
}

/* Clock management */
static int qcom_phy_enable_clocks(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy->clock_dev) {
        return 0;
    }
    
    for (uint8_t i = 0; i < phy->num_clocks; i++) {
        ret = clock_control_on(phy->clock_dev, (clock_control_subsys_t)&phy->clock_ids[i]);
        if (ret) {
            LOG_ERR("Failed to enable clock %d: %d", phy->clock_ids[i], ret);
            return ret;
        }
    }
    
    return 0;
}

static int qcom_phy_disable_clocks(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy->clock_dev) {
        return 0;
    }
    
    for (uint8_t i = 0; i < phy->num_clocks; i++) {
        ret = clock_control_off(phy->clock_dev, (clock_control_subsys_t)&phy->clock_ids[i]);
        if (ret) {
            LOG_ERR("Failed to disable clock %d: %d", phy->clock_ids[i], ret);
            return ret;
        }
    }
    
    return 0;
}

/* Reset management */
static int qcom_phy_assert_resets(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy->reset_dev) {
        return 0;
    }
    
    for (uint8_t i = 0; i < phy->num_resets; i++) {
        ret = reset_line_assert(phy->reset_dev, phy->reset_ids[i]);
        if (ret) {
            LOG_ERR("Failed to assert reset %d: %d", phy->reset_ids[i], ret);
            return ret;
        }
    }
    
    return 0;
}

static int qcom_phy_deassert_resets(struct qcom_phy_data *phy)
{
    int ret;
    
    if (!phy->reset_dev) {
        return 0;
    }
    
    for (uint8_t i = 0; i < phy->num_resets; i++) {
        ret = reset_line_deassert(phy->reset_dev, phy->reset_ids[i]);
        if (ret) {
            LOG_ERR("Failed to deassert reset %d: %d", phy->reset_ids[i], ret);
            return ret;
        }
    }
    
    return 0;
}

/* PHY API implementation */
static int qcom_phy_init(const struct device *dev)
{
    struct qcom_phy_data *phy = dev->data;
    int ret;
    
    LOG_INF("Initializing Qualcomm USB PHY (type: %d)", phy->type);
    
    if (phy->initialized) {
        return 0;
    }
    
    /* Enable clocks */
    ret = qcom_phy_enable_clocks(phy);
    if (ret) {
        return ret;
    }
    
    /* Deassert resets */
    ret = qcom_phy_deassert_resets(phy);
    if (ret) {
        qcom_phy_disable_clocks(phy);
        return ret;
    }
    
    /* Wait for clocks to stabilize */
    k_msleep(1);
    
    phy->initialized = true;
    
    LOG_INF("Qualcomm USB PHY initialized successfully");
    return 0;
}

static int qcom_phy_deinit(const struct device *dev)
{
    struct qcom_phy_data *phy = dev->data;
    
    LOG_INF("Deinitializing Qualcomm USB PHY");
    
    if (!phy->initialized) {
        return 0;
    }
    
    /* Power off PHY */
    qcom_phy_power_off(dev);
    
    /* Assert resets */
    qcom_phy_assert_resets(phy);
    
    /* Disable clocks */
    qcom_phy_disable_clocks(phy);
    
    phy->initialized = false;
    phy->powered = false;
    
    return 0;
}

static int qcom_phy_power_on(const struct device *dev)
{
    struct qcom_phy_data *phy = dev->data;
    int ret;
    
    LOG_DBG("Powering on Qualcomm USB PHY");
    
    if (phy->powered) {
        return 0;
    }
    
    if (!phy->initialized) {
        ret = qcom_phy_init(dev);
        if (ret) {
            return ret;
        }
    }
    
    /* Initialize PHY based on type */
    switch (phy->type) {
    case USB_STACK_PHY_TYPE_QUSB2:
        ret = qusb2_phy_init_sequence(phy);
        break;
        
    case USB_STACK_PHY_TYPE_QMP_USB3:
    case USB_STACK_PHY_TYPE_QMP_USB31:
        ret = qmp_usb3_phy_init_sequence(phy);
        break;
        
    default:
        LOG_ERR("Unsupported PHY type: %d", phy->type);
        return -ENOTSUP;
    }
    
    if (ret) {
        LOG_ERR("PHY initialization sequence failed: %d", ret);
        return ret;
    }
    
    phy->powered = true;
    
    LOG_INF("Qualcomm USB PHY powered on successfully");
    return 0;
}

int qcom_phy_power_off(const struct device *dev)
{
    struct qcom_phy_data *phy = dev->data;
    
    LOG_DBG("Powering off Qualcomm USB PHY");
    
    if (!phy->powered) {
        return 0;
    }
    
    /* Power down PHY based on type */
    switch (phy->type) {
    case USB_STACK_PHY_TYPE_QUSB2:
        if (phy->qusb2_base) {
            qcom_phy_writel(phy->qusb2_base, QUSB2PHY_PWR_CTRL1, 0x40);
        }
        break;
        
    case USB_STACK_PHY_TYPE_QMP_USB3:
    case USB_STACK_PHY_TYPE_QMP_USB31:
        if (phy->pcs_base) {
            qcom_phy_writel(phy->pcs_base, QPHY_POWER_DOWN_CONTROL, 0x00);
        }
        break;
        
    default:
        break;
    }
    
    phy->powered = false;
    
    return 0;
}

static int qcom_phy_set_mode(const struct device *dev, int mode)
{
    struct qcom_phy_data *phy = dev->data;
    
    LOG_DBG("Setting PHY mode to %d", mode);
    
    /* Mode setting is typically handled during initialization */
    /* This can be extended based on specific requirements */
    
    return 0;
}

static int qcom_phy_calibrate(const struct device *dev)
{
    struct qcom_phy_data *phy = dev->data;
    
    LOG_DBG("Calibrating Qualcomm USB PHY");
    
    /* Calibration is typically part of the initialization sequence */
    /* This can be extended for runtime calibration if needed */
    
    return 0;
}

static usb_stack_phy_type_t qcom_phy_get_type(const struct device *dev)
{
    struct qcom_phy_data *phy = dev->data;
    return phy->type;
}

/* PHY API structure */
static const struct usb_stack_phy_api qcom_phy_api = {
    .init = qcom_phy_init,
    .deinit = qcom_phy_deinit,
    .power_on = qcom_phy_power_on,
    .power_off = qcom_phy_power_off,
    .set_mode = qcom_phy_set_mode,
    .calibrate = qcom_phy_calibrate,
    .get_type = qcom_phy_get_type,
};

/* Device instantiation macro */
#define QCOM_PHY_INIT(n)                                                      \
    static struct qcom_phy_data qcom_phy_data_##n = {                        \
        .type = DT_INST_ENUM_IDX(n, phy_type),                              \
        .powered = false,                                                     \
        .initialized = false,                                                 \
    };                                                                        \
                                                                              \
    static const struct qcom_phy_config qcom_phy_config_##n = {              \
        .qusb2_base = DT_INST_REG_ADDR_BY_NAME(n, qusb2),                   \
        .qmp_base = DT_INST_REG_ADDR_BY_NAME(n, qmp),                       \
        .pcs_base = DT_INST_REG_ADDR_BY_NAME(n, pcs),                       \
        .type = DT_INST_ENUM_IDX(n, phy_type),                              \
    };                                                                        \
                                                                              \
    DEVICE_DT_INST_DEFINE(n, qcom_phy_init, NULL,                           \
                          &qcom_phy_data_##n, &qcom_phy_config_##n,         \
                          POST_KERNEL, CONFIG_USB_STACK_INIT_PRIORITY,       \
                          &qcom_phy_api);

DT_INST_FOREACH_STATUS_OKAY(QCOM_PHY_INIT)
