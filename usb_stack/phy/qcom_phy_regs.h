/*
 * Qualcomm USB PHY Register Definitions
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QCOM_PHY_REGS_H
#define QCOM_PHY_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* QMP PHY PCS (Physical Coding Sublayer) Register Offsets */
#define QPHY_POWER_DOWN_CONTROL         0x004
#define QPHY_START_CTRL                 0x008
#define QPHY_PCS_STATUS                 0x174
#define QPHY_PCS_READY_STATUS           0x178
#define QPHY_PCS_AUTONOMOUS_MODE_CTRL   0x0C4
#define QPHY_PCS_LFPS_RXTERM_IRQ_CLEAR  0x0C8
#define QPHY_PCS_LFPS_RXTERM_IRQ_STATUS 0x178

/* QMP PHY TX Register Offsets */
#define QSERDES_TX_HIGHZ_DRVR_EN        0x058
#define QSERDES_TX_RCV_DETECT_LVL_2     0x09C
#define QSERDES_TX_LANE_MODE_1          0x084
#define QSERDES_TX_RES_CODE_LANE_OFFSET_TX 0x054
#define QSERDES_TX_RES_CODE_LANE_OFFSET_RX 0x050

/* QMP PHY RX Register Offsets */
#define QSERDES_RX_UCDR_FASTLOCK_FO_GAIN 0x040
#define QSERDES_RX_UCDR_SO_GAIN         0x04C
#define QSERDES_RX_UCDR_FASTLOCK_COUNT_LOW 0x044
#define QSERDES_RX_UCDR_FASTLOCK_COUNT_HIGH 0x048
#define QSERDES_RX_UCDR_PI_CONTROLS     0x050
#define QSERDES_RX_UCDR_SB2_THRESH1     0x054
#define QSERDES_RX_UCDR_SB2_THRESH2     0x058
#define QSERDES_RX_UCDR_SB2_GAIN1       0x05C
#define QSERDES_RX_UCDR_SB2_GAIN2       0x060
#define QSERDES_RX_EQU_ADAPTOR_CNTRL2   0x0D8
#define QSERDES_RX_EQU_ADAPTOR_CNTRL3   0x0DC
#define QSERDES_RX_EQU_ADAPTOR_CNTRL4   0x0E0
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL1 0x0D4
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL2 0x0D8
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL3 0x0DC
#define QSERDES_RX_RX_EQU_ADAPTOR_CNTRL4 0x0E0
#define QSERDES_RX_RX_IDAC_TSETTLE_HIGH  0x0F4
#define QSERDES_RX_RX_IDAC_TSETTLE_LOW   0x0F0
#define QSERDES_RX_RX_MODE_00_HIGH4      0x1DC
#define QSERDES_RX_RX_MODE_00_HIGH3      0x1D8
#define QSERDES_RX_RX_MODE_00_HIGH2      0x1D4
#define QSERDES_RX_RX_MODE_00_HIGH       0x1D0
#define QSERDES_RX_RX_MODE_00_LOW        0x1CC
#define QSERDES_RX_RX_MODE_01_HIGH4      0x1EC
#define QSERDES_RX_RX_MODE_01_HIGH3      0x1E8
#define QSERDES_RX_RX_MODE_01_HIGH2      0x1E4
#define QSERDES_RX_RX_MODE_01_HIGH       0x1E0
#define QSERDES_RX_RX_MODE_01_LOW        0x1DC

/* Status and Control Bits */
#define PHYSTATUS                       BIT(6)
#define PCS_READY                       BIT(0)

/* QUSB2 PHY Additional Register Offsets */
#define QUSB2PHY_PORT_POWERDOWN         0x0B4
#define QUSB2PHY_PORT_UTMI_CTRL1        0x0C0
#define QUSB2PHY_PORT_UTMI_CTRL2        0x0C4
#define QUSB2PHY_PORT_TUNE1             0x080
#define QUSB2PHY_PORT_TUNE2             0x084
#define QUSB2PHY_PORT_TUNE3             0x088
#define QUSB2PHY_PORT_TUNE4             0x08C
#define QUSB2PHY_PORT_TUNE5             0x090
#define QUSB2PHY_PORT_TEST_CTRL         0x0B8
#define QUSB2PHY_PLL_AUTOPGM_CTL1       0x01C
#define QUSB2PHY_PLL_PWR_CTRL           0x018
#define QUSB2PHY_PLL_STATUS             0x038
#define QUSB2PHY_PORT_QC1               0x078
#define QUSB2PHY_PORT_QC2               0x07C

/* QUSB2 PHY Control Bits */
#define POWER_DOWN                      BIT(0)
#define PLL_LOCK_DET                    BIT(5)
#define PHY_REFCLK_DET                  BIT(4)

/* QMP PHY Common Block Additional Registers */
#define QSERDES_COM_ATB_SEL1            0x000
#define QSERDES_COM_ATB_SEL2            0x004
#define QSERDES_COM_FREQ_UPDATE         0x008
#define QSERDES_COM_BG_CTRL             0x028
#define QSERDES_COM_CLK_SELECT          0x174
#define QSERDES_COM_HSCLK_SEL           0x178
#define QSERDES_COM_INTEGLOOP_BINCODE_STATUS 0x1A0
#define QSERDES_COM_PLL_ANALOG          0x1A4
#define QSERDES_COM_CORECLK_DIV         0x1A8
#define QSERDES_COM_SW_RESET            0x1AC
#define QSERDES_COM_CORE_CLK_EN         0x1B0
#define QSERDES_COM_C_READY_STATUS      0x1B4
#define QSERDES_COM_CMN_CONFIG          0x1B8
#define QSERDES_COM_SVS_MODE_CLK_SEL    0x1BC

/* QMP PHY Lane-specific Registers */
#define QSERDES_TX_DEBUG_BUS_SEL        0x064
#define QSERDES_TX_TRANSCEIVER_BIAS_EN  0x054
#define QSERDES_TX_CLKBUF_ENABLE        0x008
#define QSERDES_TX_CMN_CONTROL_ONE      0x00C
#define QSERDES_TX_CMN_CONTROL_TWO      0x010
#define QSERDES_TX_CMN_CONTROL_THREE    0x014
#define QSERDES_TX_TX_EMP_POST1_LVL     0x018
#define QSERDES_TX_TX_DRV_LVL           0x01C
#define QSERDES_TX_RESET_TSYNC_EN       0x044
#define QSERDES_TX_PRE_STALL_LDO_BOOST_EN 0x048
#define QSERDES_TX_TX_BAND              0x024
#define QSERDES_TX_SLEW_CNTL            0x040
#define QSERDES_TX_INTERFACE_SELECT     0x04C
#define QSERDES_TX_RES_CODE_UP          0x04C
#define QSERDES_TX_RES_CODE_DN          0x050

/* QMP PHY RX Lane Registers */
#define QSERDES_RX_CDR_CONTROL          0x000
#define QSERDES_RX_CDR_CONTROL2         0x004
#define QSERDES_RX_CDR_FREEZE_UP_DN     0x008
#define QSERDES_RX_CDR_RESET_OVERRIDE   0x00C
#define QSERDES_RX_READBACK_STATUS      0x010
#define QSERDES_RX_STARTUP_OVRD_IN_CFG  0x014
#define QSERDES_RX_RX_TERM_BW           0x090
#define QSERDES_RX_VGA_CAL_CNTRL1       0x0D0
#define QSERDES_RX_VGA_CAL_CNTRL2       0x0D4
#define QSERDES_RX_GM_CAL               0x0D8
#define QSERDES_RX_RX_VGA_GAIN2_LSB     0x0F8
#define QSERDES_RX_RX_VGA_GAIN2_MSB     0x0FC
#define QSERDES_RX_SIGDET_ENABLES       0x110
#define QSERDES_RX_SIGDET_CNTRL         0x114
#define QSERDES_RX_SIGDET_LVL           0x118
#define QSERDES_RX_SIGDET_DEGLITCH_CNTRL 0x11C
#define QSERDES_RX_RX_BAND              0x120
#define QSERDES_RX_RX_INTERFACE_MODE    0x12C

/* Timing and Delay Values */
#define PHY_INIT_DELAY_US               1000
#define PLL_LOCK_TIMEOUT_US             1000
#define PHY_READY_TIMEOUT_US            150

/* Tuning Parameter Masks */
#define QUSB2PHY_TUNE_MASK              0xFF
#define QMP_PHY_TUNE_MASK               0xFF

/* Power Control Values */
#define QUSB2_POWER_DOWN_VAL            0x40
#define QUSB2_POWER_UP_VAL              0x00
#define QMP_POWER_DOWN_VAL              0x00
#define QMP_POWER_UP_VAL                0x01

/* Clock and Reset Control */
#define PHY_CLK_SCHEME_SEL              BIT(0)
#define PHY_CLK_SCHEME_48               0
#define PHY_CLK_SCHEME_27               1

/* USB3 Specific Definitions */
#define USB3_PHY_AUTONOMOUS_MODE_CTRL   0x0C4
#define USB3_PHY_LFPS_RXTERM_IRQ_CLEAR  0x0C8
#define USB3_PHY_POWER_STATE_CONFIG     0x0CC
#define USB3_PHY_SW_RESET               0x0D0
#define USB3_PHY_START                  0x0D4

/* USB2 Specific Definitions */
#define USB2_PHY_USB_PHY_UTMI_CTRL0     0x3C
#define USB2_PHY_USB_PHY_UTMI_CTRL5     0x50
#define USB2_PHY_USB_PHY_HS_PHY_CTRL1   0x60
#define USB2_PHY_USB_PHY_HS_PHY_CTRL2   0x64
#define USB2_PHY_USB_PHY_CFG0           0x94
#define USB2_PHY_USB_PHY_REFCLK_CTRL    0xA0

/* Interrupt and Status Registers */
#define QPHY_IRQ_CMD                    0x164
#define QPHY_LOCK_DETECT_CONFIG1        0x0C8
#define QPHY_LOCK_DETECT_CONFIG2        0x0CC
#define QPHY_LOCK_DETECT_CONFIG3        0x0D0
#define QPHY_POWER_STATE_CONFIG         0x0CC
#define QPHY_POWER_STATE_CONFIG2        0x0D4
#define QPHY_POWER_STATE_CONFIG4        0x0DC

/* Test and Debug Registers */
#define QPHY_DEBUG_BUS0                 0x130
#define QPHY_DEBUG_BUS1                 0x134
#define QPHY_DEBUG_BUS2                 0x138
#define QPHY_DEBUG_BUS3                 0x13C
#define QPHY_DEBUG_BUS_CLKSEL           0x140
#define QPHY_DEBUG_BUS_SEL              0x144

#ifdef __cplusplus
}
#endif

#endif /* QCOM_PHY_REGS_H */
