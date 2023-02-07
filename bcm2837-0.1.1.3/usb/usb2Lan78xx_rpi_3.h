/* usb2Lan78xx_rpi_3.h - Generation 2 USB Lan78xx series driver */

/*
 * Copyright (c) 2019 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
modification history
--------------------
20may19,hkl  created (F11409)
*/

#ifndef __INCusb2Lan78xx_rpi_3h
#define __INCusb2Lan78xx_rpi_3h

/* includes */

#include <vxWorks.h>
#include <iosLib.h>
#include <usb2Helper.h>
#include <usb2End.h>

#ifdef __cplusplus
extern "C" {
#endif


/* defines */
#define USB2_END_LAN78XX            (0xF0)

#define LAN78XX_CHIP_ID_7800        0x7800u
#define LAN78XX_CHIP_ID_7850        0x7850u

#define BIT(nr)                     (1u << (nr))

/* LAN78XX System Control and Status Registers */

#define LAN78XX_INT_STS             0x00Cu
#define LAN78XX_HW_CFG              0x010u
#define LAN78XX_PMT_CTL             0x014u
#define LAN78XX_E2P_CMD             0x040u
#define LAN78XX_E2P_DATA            0x044u
#define LAN78XX_USB_CFG0            0x080u
#define LAN78XX_BURST_CAP           0x090u
#define LAN78XX_BULK_IN_DLY         0x094u
#define LAN78XX_RFE_CTL             0x0B0u
#define LAN78XX_FCT_RX_CTL          0x0C0u
#define LAN78XX_FCT_TX_CTL          0x0C4u
#define LAN78XX_FCT_RX_FIFO_END     0x0C8u
#define LAN78XX_FCT_TX_FIFO_END     0x0CCu
#define LAN78XX_MAC_CR              0x100u
#define LAN78XX_MAC_RX              0x104u
#define LAN78XX_MAC_TX              0x108u
#define LAN78XX_FLOW                0x10Cu
#define LAN78XX_RX_ADDRH            0x118u
#define LAN78XX_RX_ADDRL            0x11Cu
#define LAN78XX_MII_ACC             0x120u
#define LAN78XX_MII_DATA            0x124u
#define LAN78XX_RX_ADDR_FLTH(n)     (0x400u + 8u * (n))
#define LAN78XX_RX_ADDR_FLTL(n)     (0x404u + 8u * (n))

#define LAN78XX_HW_CFG_LED1_EN_BIT  21u
#define LAN78XX_HW_CFG_LED0_EN_BIT  20u
#define LAN78XX_HW_CFG_LRST_BIT     1u

#define LAN78XX_PMT_CTL_READY_BIT   7u
#define LAN78XX_PMT_CTL_PHY_RST_BIT 4u

#define LAN78XX_E2P_CMD_EPC_BUSY_BIT    31u
#define LAN78XX_E2P_CMD_OP_BIT          28u
#define LAN78XX_EPC_CMD_READ_CMD        (0x0u << LAN78XX_E2P_CMD_OP_BIT)
#define LAN78XX_E2P_CMD_EPC_ADDR_MASK   0x1FFu

#define LAN78XX_USB_CFG0_BIR_BIT        6u

#define LAN78XX_RX_ADDR_FLTH_VALID_BIT  31u

#define LAN78XX_RFE_CTL_BCAST_EN_BIT    10u
#define LAN78XX_RFE_CTL_MCAST_EN_BIT    9u
#define LAN78XX_RFE_CTL_UCAST_EN_BIT    8u
#define LAN78XX_RFE_CTL_DA_PERFECT_BIT  1u

#define LAN78XX_MAC_CR_ADP_BIT          13u
#define LAN78XX_MAC_CR_AUTO_DUPLEX_BIT  12u
#define LAN78XX_MAC_CR_AUTO_SPEED_BIT   11u

#define LAN78XX_MAC_RX_FCS_STRIP_BIT    4u
#define LAN78XX_MAC_RX_RXEN_BIT         0u

#define LAN78XX_MAC_TX_TXEN_BIT         0u

#define LAN78XX_FCT_RX_CTL_EN_BIT       31u
#define LAN78XX_FCT_TX_CTL_EN_BIT       31u

#define LAN78XX_MII_ACC_MII_READ        0x00000000u
#define LAN78XX_MII_ACC_MII_WRITE       0x00000002u
#define LAN78XX_MII_ACC_PHYADDR_BIT     11u
#define LAN78XX_MII_ACC_REG_BIT         6u
#define LAN78XX_MII_ACC_BUSY_BIT        0u

/* LAN78XX OTP registers */

#define LAN78XX_OTP_PWR_DN              0x1000u
#define LAN78XX_OTP_ADDR1               0x1004u
#define LAN78XX_OTP_ADDR2               0x1008u
#define LAN78XX_OTP_RD_DATA             0x1018u
#define LAN78XX_OTP_FUNC_CMD            0x1020u
#define LAN78XX_OTP_CMD_GO              0x1028u
#define LAN78XX_OTP_STATUS              0x1030u

#define LAN78XX_OTP_PWR_DN_PWRDN_N_BIT  0u
#define LAN78XX_OTP_ADDR1_MASK          0x1Fu
#define LAN78XX_OTP_ADDR2_MASK          0xFFu
#define LAN78XX_OTP_FUNC_CMD_BIT        0u
#define LAN78XX_OTP_FUNC_CMD_READ       (1u << LAN78XX_OTP_FUNC_CMD_BIT)
#define LAN78XX_OTP_CMD_GO_GO_BIT       0u
#define LAN78XX_OTP_STATUS_BUSY_BIT     0u

/* LAN78XX PHY Registers */

#define LAN78XX_PHY_MCR             0x0u
#define LAN78XX_PHY_MSR             0x1u
#define LAN78XX_PHY_AN_ADV          0x4u
#define LAN78XX_PHY_1000BASE_T_CTRL 0x9u

#define LAN78XX_PHY_MSR_LINK_BIT    2u

/* LAN78XX PHY address */

#define USB2_LAN78XX_PHY_ADDR       0x1u    /* fixed integrated PHY address */


#define USB2_END_LAN78XX_HEADER             0x8u
#define USB2_LAN78XX_LINK_STATUS_OFFSET     0x0u
#define USB2_LAN78XX_LINK_STATUS_MASK       (BIT(LAN78XX_PHY_MSR_LINK_BIT))

#define USB2_LAN78XX_TXCOMMANDA_LEN_MASK    0xFFFFFu
#define USB2_LAN78XX_TXCOMMANDA_FCS_BIT     22u
#define USB2_LAN78XX_RXCOMMANDA_LEN_MASK    0X3FFFu
#define USB2_LAN78XX_RXCOMMANDA_RXE_BIT     18u

#define LAN78XX_MAX_RX_FIFO_SIZE            (12 * 1024)
#define LAN78XX_MAX_TX_FIFO_SIZE            (12 * 1024)

#define LAN78XX_MAC_RX_MAX_SIZE             (1536 + 4 + 4)

/* USB Vendor Requests */

#define USB2_LAN78XX_SET_REGISTER_VENDOR_RQ 0xA0u
#define USB2_LAN78XX_GET_REGISTER_VENDOR_RQ 0xA1u

/* Forward declarations */

extern void usb2Lan78xxDeInit
    (
    void
    );
extern STATUS usb2Lan78xxInit
    (
    char * pName 
    );

#ifdef __cplusplus
}
#endif

#endif  /* __INCusb2Lan78xx_rpi_3h */

