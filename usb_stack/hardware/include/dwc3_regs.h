/*
 * DWC3 USB Controller Register Definitions
 * Copyright (c) 2025
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DWC3_REGS_H
#define DWC3_REGS_H

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DWC3 Global Registers */
#define DWC3_GSBUSCFG0          0xC100
#define DWC3_GSBUSCFG1          0xC104
#define DWC3_GTXTHRCFG          0xC108
#define DWC3_GRXTHRCFG          0xC10C
#define DWC3_GCTL               0xC110
#define DWC3_GEVTEN             0xC114
#define DWC3_GSTS               0xC118
#define DWC3_GUCTL1             0xC11C
#define DWC3_GSNPSID            0xC120
#define DWC3_GGPIO              0xC124
#define DWC3_GUID               0xC128
#define DWC3_GUCTL              0xC12C
#define DWC3_GBUSERRADDR0       0xC130
#define DWC3_GBUSERRADDR1       0xC134
#define DWC3_GPRTBIMAP0         0xC138
#define DWC3_GPRTBIMAP1         0xC13C
#define DWC3_GHWPARAMS0         0xC140
#define DWC3_GHWPARAMS1         0xC144
#define DWC3_GHWPARAMS2         0xC148
#define DWC3_GHWPARAMS3         0xC14C
#define DWC3_GHWPARAMS4         0xC150
#define DWC3_GHWPARAMS5         0xC154
#define DWC3_GHWPARAMS6         0xC158
#define DWC3_GHWPARAMS7         0xC15C
#define DWC3_GDBGFIFOSPACE      0xC160
#define DWC3_GDBGLTSSM          0xC164
#define DWC3_GDBGLNMCC          0xC168
#define DWC3_GDBGBMU            0xC16C
#define DWC3_GDBGLSPMUX         0xC170
#define DWC3_GDBGLSP            0xC174
#define DWC3_GDBGEPINFO0        0xC178
#define DWC3_GDBGEPINFO1        0xC17C
#define DWC3_GPRTBIMAP_HS0      0xC180
#define DWC3_GPRTBIMAP_HS1      0xC184
#define DWC3_GPRTBIMAP_FS0      0xC188
#define DWC3_GPRTBIMAP_FS1      0xC18C

/* DWC3 Device Registers */
#define DWC3_DCFG               0xC700
#define DWC3_DCTL               0xC704
#define DWC3_DEVTEN             0xC708
#define DWC3_DSTS               0xC70C
#define DWC3_DGCMDPAR           0xC710
#define DWC3_DGCMD              0xC714
#define DWC3_DALEPENA           0xC720

/* DWC3 Device Endpoint Registers */
#define DWC3_DEP_BASE(n)        (0xC800 + (n * 0x10))
#define DWC3_DEPCMDPAR2(n)      (DWC3_DEP_BASE(n) + 0x00)
#define DWC3_DEPCMDPAR1(n)      (DWC3_DEP_BASE(n) + 0x04)
#define DWC3_DEPCMDPAR0(n)      (DWC3_DEP_BASE(n) + 0x08)
#define DWC3_DEPCMD(n)          (DWC3_DEP_BASE(n) + 0x0C)

/* DWC3 OTG Registers */
#define DWC3_OCFG               0xCC00
#define DWC3_OCTL               0xCC04
#define DWC3_OEVT               0xCC08
#define DWC3_OEVTEN             0xCC0C
#define DWC3_OSTS               0xCC10

/* Global Configuration Register (GCTL) */
#define DWC3_GCTL_PWRDNSCALE_MASK       GENMASK(31, 19)
#define DWC3_GCTL_U2RSTECN              BIT(16)
#define DWC3_GCTL_RAMCLKSEL_MASK        GENMASK(7, 6)
#define DWC3_GCTL_CLK_BUS               (0 << 6)
#define DWC3_GCTL_CLK_PIPE              (1 << 6)
#define DWC3_GCTL_CLK_PIPEHALF          (2 << 6)
#define DWC3_GCTL_PRTCAP_MASK           GENMASK(13, 12)
#define DWC3_GCTL_PRTCAP_HOST           (1 << 12)
#define DWC3_GCTL_PRTCAP_DEVICE         (2 << 12)
#define DWC3_GCTL_PRTCAP_OTG            (3 << 12)
#define DWC3_GCTL_CORESOFTRESET         BIT(11)
#define DWC3_GCTL_SOFITPSYNC            BIT(10)
#define DWC3_GCTL_SCALEDOWN_MASK        GENMASK(5, 4)
#define DWC3_GCTL_SCALEDOWN(n)          ((n) << 4)
#define DWC3_GCTL_DISSCRAMBLE           BIT(3)
#define DWC3_GCTL_U2EXIT_LFPS           BIT(2)
#define DWC3_GCTL_GBLHIBERNATIONEN      BIT(1)
#define DWC3_GCTL_DSBLCLKGTNG           BIT(0)

/* Global Status Register (GSTS) */
#define DWC3_GSTS_OTG_IP                BIT(10)
#define DWC3_GSTS_BC_IP                 BIT(9)
#define DWC3_GSTS_ADP_IP                BIT(8)
#define DWC3_GSTS_HOST_IP               BIT(7)
#define DWC3_GSTS_DEVICE_IP             BIT(6)
#define DWC3_GSTS_CSR_TIMEOUT           BIT(5)
#define DWC3_GSTS_BUS_ERR_ADDR_VLD      BIT(4)
#define DWC3_GSTS_CURMOD_MASK           GENMASK(1, 0)
#define DWC3_GSTS_CURMOD_DEVICE         (0 << 0)
#define DWC3_GSTS_CURMOD_HOST           (1 << 0)

/* Device Configuration Register (DCFG) */
#define DWC3_DCFG_DEVADDR_MASK          GENMASK(31, 25)
#define DWC3_DCFG_DEVADDR(addr)         ((addr) << 25)
#define DWC3_DCFG_SPEED_MASK            GENMASK(2, 0)
#define DWC3_DCFG_SUPERSPEED_PLUS       (5 << 0)
#define DWC3_DCFG_SUPERSPEED            (4 << 0)
#define DWC3_DCFG_HIGHSPEED             (0 << 0)
#define DWC3_DCFG_FULLSPEED2            (1 << 0)
#define DWC3_DCFG_LOWSPEED              (2 << 0)
#define DWC3_DCFG_FULLSPEED1            (3 << 0)
#define DWC3_DCFG_LPM_CAP               BIT(22)
#define DWC3_DCFG_NUMP_SHIFT            17
#define DWC3_DCFG_NUMP(n)               (((n) >> DWC3_DCFG_NUMP_SHIFT) & 0x1f)
#define DWC3_DCFG_DEVADDR_SHIFT         25

/* Device Control Register (DCTL) */
#define DWC3_DCTL_RUN_STOP              BIT(31)
#define DWC3_DCTL_CSFTRST               BIT(30)
#define DWC3_DCTL_LSFTRST               BIT(29)
#define DWC3_DCTL_HIRD_THRES_MASK       GENMASK(28, 24)
#define DWC3_DCTL_HIRD_THRES(n)         ((n) << 24)
#define DWC3_DCTL_APPL1RES              BIT(23)
#define DWC3_DCTL_TRGTULST_MASK         GENMASK(20, 17)
#define DWC3_DCTL_TRGTULST(n)           ((n) << 17)
#define DWC3_DCTL_TRGTULST_U2           (DWC3_DCTL_TRGTULST(2))
#define DWC3_DCTL_TRGTULST_U3           (DWC3_DCTL_TRGTULST(3))
#define DWC3_DCTL_TRGTULST_SS_DIS       (DWC3_DCTL_TRGTULST(4))
#define DWC3_DCTL_TRGTULST_RX_DET       (DWC3_DCTL_TRGTULST(5))
#define DWC3_DCTL_TRGTULST_SS_INACT     (DWC3_DCTL_TRGTULST(6))
#define DWC3_DCTL_INITU2ENA             BIT(12)
#define DWC3_DCTL_ACCEPTU2ENA           BIT(11)
#define DWC3_DCTL_INITU1ENA             BIT(10)
#define DWC3_DCTL_ACCEPTU1ENA           BIT(9)
#define DWC3_DCTL_TSTCTRL_MASK          GENMASK(7, 4)
#define DWC3_DCTL_ULSTCHNGREQ_MASK      GENMASK(8, 5)
#define DWC3_DCTL_ULSTCHNGREQ(n)        (((n) << 5) & DWC3_DCTL_ULSTCHNGREQ_MASK)
#define DWC3_DCTL_ULSTCHNG_NO_ACTION    (DWC3_DCTL_ULSTCHNGREQ(0))
#define DWC3_DCTL_ULSTCHNG_SS_DISABLED  (DWC3_DCTL_ULSTCHNGREQ(4))
#define DWC3_DCTL_ULSTCHNG_RX_DETECT    (DWC3_DCTL_ULSTCHNGREQ(5))
#define DWC3_DCTL_ULSTCHNG_SS_INACTIVE  (DWC3_DCTL_ULSTCHNGREQ(6))
#define DWC3_DCTL_ULSTCHNG_RECOVERY     (DWC3_DCTL_ULSTCHNGREQ(8))
#define DWC3_DCTL_ULSTCHNG_COMPLIANCE   (DWC3_DCTL_ULSTCHNGREQ(10))
#define DWC3_DCTL_ULSTCHNG_LOOPBACK     (DWC3_DCTL_ULSTCHNGREQ(11))

/* Device Status Register (DSTS) */
#define DWC3_DSTS_DCNRD                 BIT(29)
#define DWC3_DSTS_SRE                   BIT(28)
#define DWC3_DSTS_RSS                   BIT(25)
#define DWC3_DSTS_SSS                   BIT(24)
#define DWC3_DSTS_COREIDLE              BIT(23)
#define DWC3_DSTS_DEVCTRLHLT            BIT(22)
#define DWC3_DSTS_USBLNKST_MASK         GENMASK(21, 18)
#define DWC3_DSTS_USBLNKST(n)           (((n) & DWC3_DSTS_USBLNKST_MASK) >> 18)
#define DWC3_DSTS_RXFIFOEMPTY           BIT(17)
#define DWC3_DSTS_SOFFN_MASK            GENMASK(16, 3)
#define DWC3_DSTS_SOFFN(n)              (((n) & DWC3_DSTS_SOFFN_MASK) >> 3)
#define DWC3_DSTS_CONNECTSPD            GENMASK(2, 0)

/* Device Endpoint Command Register (DEPCMD) */
#define DWC3_DEPCMD_COMMANDPARAM_MASK   GENMASK(31, 16)
#define DWC3_DEPCMD_COMMANDPARAM(n)     ((n) << 16)
#define DWC3_DEPCMD_CMDACT              BIT(10)
#define DWC3_DEPCMD_CMDIOC              BIT(8)
#define DWC3_DEPCMD_CMDTYP_MASK         GENMASK(7, 0)
#define DWC3_DEPCMD_DEPSTARTCFG         (0x09)
#define DWC3_DEPCMD_ENDTRANSFER         (0x08)
#define DWC3_DEPCMD_UPDATETRANSFER      (0x07)
#define DWC3_DEPCMD_STARTTRANSFER       (0x06)
#define DWC3_DEPCMD_CLEARSTALL          (0x05)
#define DWC3_DEPCMD_SETSTALL            (0x04)
#define DWC3_DEPCMD_GETEPSTATE          (0x03)
#define DWC3_DEPCMD_SETTRANSFRESOURCE   (0x02)
#define DWC3_DEPCMD_SETEPCONFIG         (0x01)

/* Endpoint Configuration Parameters */
#define DWC3_DEPCFG_EP_TYPE_MASK        GENMASK(2, 1)
#define DWC3_DEPCFG_EP_TYPE(n)          ((n) << 1)
#define DWC3_DEPCFG_MAX_PACKET_SIZE_MASK GENMASK(26, 16)
#define DWC3_DEPCFG_MAX_PACKET_SIZE(n)  ((n) << 16)
#define DWC3_DEPCFG_EP_NUMBER_MASK      GENMASK(7, 4)
#define DWC3_DEPCFG_EP_NUMBER(n)        ((n) << 4)
#define DWC3_DEPCFG_FIFO_NUMBER_MASK    GENMASK(21, 17)
#define DWC3_DEPCFG_FIFO_NUMBER(n)      ((n) << 17)
#define DWC3_DEPCFG_XFER_COMPLETE_EN    BIT(8)
#define DWC3_DEPCFG_XFER_NOT_READY_EN   BIT(9)

/* Event Buffer Registers */
#define DWC3_GEVNTADRLO(n)              (0xC400 + (n * 0x10))
#define DWC3_GEVNTADRHI(n)              (0xC404 + (n * 0x10))
#define DWC3_GEVNTSIZ(n)                (0xC408 + (n * 0x10))
#define DWC3_GEVNTCOUNT(n)              (0xC40C + (n * 0x10))

/* Device Event Enable Register (DEVTEN) */
#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN   BIT(12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN      BIT(11)
#define DWC3_DEVTEN_CMDCMPLTEN          BIT(10)
#define DWC3_DEVTEN_ERRTICERREN         BIT(9)
#define DWC3_DEVTEN_SOFEN               BIT(7)
#define DWC3_DEVTEN_EOPFEN              BIT(6)
#define DWC3_DEVTEN_HIBERNATIONREQEVTEN BIT(5)
#define DWC3_DEVTEN_WKUPEVTEN           BIT(4)
#define DWC3_DEVTEN_ULSTCNGEN           BIT(3)
#define DWC3_DEVTEN_CONNECTDONEEN       BIT(2)
#define DWC3_DEVTEN_USBRSTEN            BIT(1)
#define DWC3_DEVTEN_DISCONNEVTEN        BIT(0)

/* Device Event Types */
#define DWC3_DEVICE_EVENT_DISCONNECT    0
#define DWC3_DEVICE_EVENT_RESET         1
#define DWC3_DEVICE_EVENT_CONNECT_DONE  2
#define DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE 3
#define DWC3_DEVICE_EVENT_WAKEUP        4
#define DWC3_DEVICE_EVENT_HIBER_REQ     5
#define DWC3_DEVICE_EVENT_EOPF          6
#define DWC3_DEVICE_EVENT_SOF           7
#define DWC3_DEVICE_EVENT_ERRATIC_ERROR 9
#define DWC3_DEVICE_EVENT_CMD_CMPL      10
#define DWC3_DEVICE_EVENT_OVERFLOW      11

/* Endpoint Event Types */
#define DWC3_DEPEVT_XFERCOMPLETE        0x01
#define DWC3_DEPEVT_XFERINPROGRESS      0x02
#define DWC3_DEPEVT_XFERNOTREADY        0x03
#define DWC3_DEPEVT_RXTXFIFOEVT         0x04
#define DWC3_DEPEVT_STREAMEVT           0x06
#define DWC3_DEPEVT_EPCMDCMPLT          0x07

/* TRB Control Fields */
#define DWC3_TRB_CTRL_HWO               BIT(0)
#define DWC3_TRB_CTRL_LST               BIT(1)
#define DWC3_TRB_CTRL_CHN               BIT(2)
#define DWC3_TRB_CTRL_CSP               BIT(3)
#define DWC3_TRB_CTRL_TRBCTL_MASK       GENMASK(9, 4)
#define DWC3_TRB_CTRL_TRBCTL(n)         ((n) << 4)
#define DWC3_TRB_CTRL_ISP_IMI           BIT(10)
#define DWC3_TRB_CTRL_IOC               BIT(11)
#define DWC3_TRB_CTRL_SID_SOFN_MASK     GENMASK(31, 14)
#define DWC3_TRB_CTRL_SID_SOFN(n)       ((n) << 14)

/* TRB Types */
#define DWC3_TRBCTL_NORMAL              1
#define DWC3_TRBCTL_CONTROL_SETUP       2
#define DWC3_TRBCTL_CONTROL_STATUS2     3
#define DWC3_TRBCTL_CONTROL_STATUS3     4
#define DWC3_TRBCTL_CONTROL_DATA        5
#define DWC3_TRBCTL_ISOCHRONOUS_FIRST   6
#define DWC3_TRBCTL_ISOCHRONOUS         7
#define DWC3_TRBCTL_LINK_TRB            8

/* Hardware Parameters */
#define DWC3_GHWPARAMS0_MODE_MASK       GENMASK(2, 0)
#define DWC3_GHWPARAMS0_MODE_GADGET     0
#define DWC3_GHWPARAMS0_MODE_HOST       1
#define DWC3_GHWPARAMS0_MODE_DRD        2
#define DWC3_GHWPARAMS0_MBUS_TYPE_MASK  GENMASK(5, 3)
#define DWC3_GHWPARAMS0_SBUS_TYPE_MASK  GENMASK(8, 6)
#define DWC3_GHWPARAMS0_MDWIDTH_MASK    GENMASK(15, 8)
#define DWC3_GHWPARAMS0_SDWIDTH_MASK    GENMASK(23, 16)
#define DWC3_GHWPARAMS0_AWIDTH_MASK     GENMASK(31, 24)

#define DWC3_GHWPARAMS1_EN_PWROPT_MASK  GENMASK(25, 24)
#define DWC3_GHWPARAMS1_EN_PWROPT_NO    0
#define DWC3_GHWPARAMS1_EN_PWROPT_CLK   1
#define DWC3_GHWPARAMS1_EN_PWROPT_HIB   2
#define DWC3_GHWPARAMS1_PWROPT(n)       ((n) << 24)

/* Maximum number of endpoints */
#define DWC3_ENDPOINTS_NUM              32

/* Event Buffer Size */
#define DWC3_EVENT_BUFFERS_SIZE         4096
#define DWC3_EVENT_SIZE                 4       /* bytes */
#define DWC3_EVENT_MAX_NUM              (DWC3_EVENT_BUFFERS_SIZE / DWC3_EVENT_SIZE)

/* TRB (Transfer Request Block) */
struct dwc3_trb {
    uint32_t bpl;
    uint32_t bph;
    uint32_t size;
    uint32_t ctrl;
} __packed;

/* Event Buffer */
struct dwc3_event_buffer {
    void *buf;
    void *cache;
    unsigned int length;
    unsigned int lpos;
    unsigned int count;
    unsigned int flags;
#define DWC3_EVENT_PENDING      BIT(0)
    uintptr_t dma;
    struct dwc3 *dwc;
};

/* Endpoint Direction */
#define DWC3_EP_DIR_IN          1
#define DWC3_EP_DIR_OUT         0

/* Endpoint Numbers */
#define DWC3_EP0_IN             1
#define DWC3_EP0_OUT            0

#ifdef __cplusplus
}
#endif

#endif /* DWC3_REGS_H */
