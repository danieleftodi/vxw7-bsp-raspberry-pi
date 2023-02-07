/* vxbFdtUsbPhyAltSocGen5_rpi_3.h - Raspberry Pi 3 USB PHY definitions */

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

#ifndef __INCvxbFdtUsbPhyAltSocGen5_rpi_3h
#define __INCvxbFdtUsbPhyAltSocGen5_rpi_3h

#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtLib.h>
#include <vxbUsbPhyLib.h>

#ifdef _WRS_CONFIG_BOOT_TIMING
#include <bootTiming.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef _WRS_CONFIG_BOOT_TIMING
#define BTT(str) 
#endif

#define ALT_SOC_GEN5_USB_PHY_NAME "rpi3-usb-phy"

#define USB_DWC_GOTGCTL        0x000
#define USB_DWC_GOTGINT        0x004
#define USB_DWC_GAHBCFG        0x008
#define USB_DWC_GUSBCFG        0x00C
#define USB_DWC_GRSTCTL        0x010
#define USB_DWC_GINTSTS        0x014
#define USB_DWC_GINTMSK        0x018
#define USB_DWC_GHWCFG1        0x044
#define USB_DWC_GHWCFG2        0x048
#define USB_DWC_GHWCFG3        0x04c
#define USB_DWC_GHWCFG4        0x050
#define USB_DWC_HPTXFSIZ       0x100

#define USB_DWC_HCFG           0x400
#define USB_DWC_HPRT           0x440
#define USB_DWC_PCGCCTL        0xe00

#define USB_DWC_DPTX_FSIZ_DIPTXF(x)         (0x104 + (x) * 4)    /* 15 => x > 1 */

#define USB_DWC_AHBCFG_BURST_LEN(x)         ((UINT32)(x << 1))

#define USB_DWC_GAHBCFG_INT_DMA_BURST_INCR  1

#define USB_DWC_USBCFG_FRC_HST_MODE         (1U << 29)
#define USB_DWC_USBCFG_TERM_SEL_DL_PULSE    (1U << 22)
#define USB_DWC_USBCFG_ULPI_EXT_VBUS_DRV    (1U << 20)
#define USB_DWC_USBCFG_ULPI_CLK_SUS_M       (1U << 19)
#define USB_DWC_USBCFG_ULPI_FSLS            (1U << 17)
#define USB_DWC_USBCFG_HNP_CAP              (1U << 9)
#define USB_DWC_USBCFG_SRP_CAP              (1U << 8)
#define USB_DWC_USBCFG_DDRSEL               (1U << 7)
#define USB_DWC_USBCFG_ULPI_UTMI_SEL        (1U << 4)
#define USB_DWC_USBCFG_PHYIF                (1U << 3)

#define USB_DWC_AHBCFG_DMA_ENA              (1U << 5)
#define USB_DWC_AHBCFG_GLBL_INT_MASK        (1U << 0)

#define USB_DWC_INTMSK_WKP                  (1U << 31)
#define USB_DWC_INTMSK_NEW_SES_DET          (1U << 30)
#define USB_DWC_INTMSK_SES_DISCON_DET       (1U << 29)
#define USB_DWC_INTMSK_CON_ID_STS_CHG       (1U << 28)
#define USB_DWC_INTMSK_USB_SUSP             (1U << 11)
#define USB_DWC_INTMSK_RXFIFO_NOT_EMPT      (1U << 4)
#define USB_DWC_INTMSK_OTG                  (1U << 2)
#define USB_DWC_INTMSK_MODE_MISMTC          (1U << 1)

#define USB_DWC_RSTCTL_AHB_IDLE             (1U << 31)
#define USB_DWC_RSTCTL_SFT_RST              (1U << 1)

#define USB_PHY_RST_AHB_IDLE_TIMEOUT        (100000)
#define USB_PHY_RST_TIMEOUT                 (10000)

typedef struct alt_soc_gen5_usb_phy_ctrl
    {
    VXB_DEV_ID       pDev;
    void *           regBase;
    void *           regHandle;
    } ALT_SOC_GEN5_USB_PHY_CTRL;

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbFdtUsbPhyAltSocGen5_rpi_3h */
