/* vxbFdtUsbPhyAltSocGen5_rpi_3.c - system specific USB controller driver 
 * porting routines
 */

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

/*
DESCRIPTION

This file defines a skeleton of functions to be used for accessing
USB module regsiters and to support USB stack with some basic hardware
initialization. 

The name of each function in this group begins with "usb" to represent
"Device Driver Services."

EXTERNAL INTERFACE
The driver provides a vxBus gen2 external interface. Initialization 
methods provided by vxbFdtUsbPhyRpi3Drv register the driver with 
VxBus. The BSP's dts entry must specify the following parameters:

\is
\i <compatible>
Specify the programming model for the controller. It should be set to 
"altr,usb-phy" and is used by vxBus gen2 for device driver selection.
\ie

\is
\i <reg>
Specifies the base address where the controller's CSR registers are 
mapped into the host's address space. All register offsets are computed
 relative to this address.
\ie

\is
\i <phy_type>
Specifies the PHY's interface type. Available types are "ulpi", "utmi",
 "utmi_wide" and "serial".
\ie

To add the driver to the vxWorks image, add the following component to the
kernel configuration:

\cs
vxprj component add INCLUDE_USB_PHY_RPI_3
\ce
*/

/* includes */

#include <vxWorks.h>
#include <vxbFdtUsbPhyAltSocGen5_rpi_3.h>

/* defines */

/* controller read and write interface */

#define USB_PHY_BAR(pPhyDev)       \
            ((ALT_SOC_GEN5_USB_PHY_CTRL *)(pPhyDev->pDevCtlr))->regBase

#define USB_PHY_HANDLE(pPhyDev)    \
            ((ALT_SOC_GEN5_USB_PHY_CTRL *)(pPhyDev->pDevCtlr))->regHandle

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#else
#    define SWAP32 
#endif /* ARMBE8 */

#undef CSR_READ_4
#define CSR_READ_4(pPhyDev, addr)                   \
        SWAP32(vxbRead32(USB_PHY_HANDLE(pPhyDev),   \
                  (UINT32 *)((char *)USB_PHY_BAR(pPhyDev) + addr)))

#undef CSR_WRITE_4
#define CSR_WRITE_4(pPhyDev, addr, data)      \
        vxbWrite32(USB_PHY_HANDLE(pPhyDev),    \
        (UINT32 *)((char *)USB_PHY_BAR(pPhyDev) + addr), SWAP32(data))

/* forward declarations */

/* vxbus methods */

LOCAL STATUS usbAltSocGen5USBPHYProbe
    (
    VXB_DEV_ID pDev
    );

LOCAL STATUS usbAltSocGen5USBPHYAttach
    (
    VXB_DEV_ID pDev
    );

LOCAL STATUS usbAltSocGen5USBPHYEnable
    (
    USB_PHY_DEV * pPhyDev
    );

LOCAL VXB_DRV_METHOD vxbFdtUsbPhyAltSocGen5MethodList[] =
    {
    { VXB_DEVMETHOD_CALL(vxbDevProbe),  usbAltSocGen5USBPHYProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach), usbAltSocGen5USBPHYAttach },
    VXB_DEVMETHOD_END
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY vxbFdtUsbPhyAltSocGen5_match[] =
    {
        {
        "brcm,bcm2708-usb-phy",     /* compatible */
        (void *) NULL
        },
        {}
    };

/* locals */

LOCAL void usbAltSocGen5USBPHYInitInternal
    (
    USB_PHY_DEV * pPhyDev
    );

/* externs */

IMPORT void sysMsDelay (UINT32);
IMPORT void sysUsDelay (UINT32);

IMPORT USB_PHY_TYPE vxbFdtUsbPhyTypeGet
    (
    struct vxbDev * pDev
    );

VXB_DRV vxbFdtUsbPhyRpi3Drv =
    {
    { NULL } ,
    ALT_SOC_GEN5_USB_PHY_NAME,      /* Name */
    "RPI3 USB PHY FDT driver",      /* Description */
    VXB_BUSID_FDT,                  /* Class */
    0,                              /* Flags */
    0,                              /* Reference count */
    vxbFdtUsbPhyAltSocGen5MethodList/* Method table */
    };

VXB_DRV_DEF(vxbFdtUsbPhyRpi3Drv)

/******************************************************************************
*
* usbAltSocGen5USBPHYProbe - probe a valid hardware
*
* This routine is used to probe a valid hardware.
*
* RETURNS: OK, or ERROR if there is anything wrong.
*
* ERRNO: N/A
*/

LOCAL STATUS usbAltSocGen5USBPHYProbe
    (
    VXB_DEV_ID pDev
    )
    {
    return vxbFdtDevMatch (pDev, vxbFdtUsbPhyAltSocGen5_match, NULL);
    }

/******************************************************************************
*
* usbAltSocGen5USBPHYProbe - attach the device to the driver
*
* This routine is used to attach the controller to the driver.
*
* RETURNS: OK, or ERROR if there is anything wrong.
*
* ERRNO: N/A
*/

LOCAL STATUS usbAltSocGen5USBPHYAttach
    (
    VXB_DEV_ID pDev
    )
    {
    STATUS              status     = ERROR;
    VXB_RESOURCE *      pRegRes    = NULL;
    VXB_RESOURCE_ADR *  pRegResAdr = NULL;
    USB_PHY_DEV *       pPhyDev    = NULL;

    ALT_SOC_GEN5_USB_PHY_CTRL * pDevCtlr = NULL;

    if (pDev == NULL)
        goto exit;

    pDevCtlr = (ALT_SOC_GEN5_USB_PHY_CTRL*)
                        vxbMemAlloc(sizeof(ALT_SOC_GEN5_USB_PHY_CTRL));
    if (pDevCtlr == NULL)
        goto exit;

    pRegRes = vxbResourceAlloc(pDev, VXB_RES_MEMORY, 0);
    if(pRegRes == NULL)
        goto exit;

    pRegResAdr = (VXB_RESOURCE_ADR *)pRegRes->pRes;
    pDevCtlr->regBase = (void *)pRegResAdr->virtual;
    pDevCtlr->regHandle = pRegResAdr->pHandle;

    pPhyDev = (USB_PHY_DEV*)vxbMemAlloc(sizeof(USB_PHY_DEV));
    if (pPhyDev == NULL)
        goto exit;

    pPhyDev->pDev = pDev;
    vxbDevSoftcSet(pDev, (void*)pPhyDev);

    pPhyDev->pDevCtlr = (void*)pDevCtlr;

    /* Record the phy tpye */

    pPhyDev->phyType = vxbFdtUsbPhyTypeGet(pDev);

    pPhyDev->phyEnable = usbAltSocGen5USBPHYEnable;

    status = vxbUsbPhyAdd(pPhyDev);
exit:
    if (status != OK)
        {
        if (pRegRes != NULL)
            (void)vxbResourceFree (pDev, pRegRes);

        if (pDevCtlr != NULL)
            vxbMemFree(pDevCtlr);
            
        if (pPhyDev != NULL)
            vxbMemFree(pPhyDev);
        }
    return status;
    }

LOCAL STATUS usbAltSocGen5USBPHYEnable
    (
    USB_PHY_DEV * pPhyDev
    )
    {
    STATUS status = ERROR;

    if (pPhyDev == NULL)
        goto exit;
    
    (void)usbAltSocGen5USBPHYInitInternal(pPhyDev);

    status = OK;
exit:
    return status;
    }

/*******************************************************************************
*
* usbAltSocGen5USBPHYReset - reset the USB PHY for alt_soc_gen5
*
* This routine reset USB PHY for alt_soc_gen5.
*
* RETURNS: N/A.
*
* ERRNO: N/A
*/

LOCAL void usbAltSocGen5USBPHYReset
    (
    USB_PHY_DEV * pPhyDev
    )
    {
    UINT32 tempRst = 0;
    int i = 0;

    /* Wait for AHB master IDLE state. */
    
    BTT(">usbAltSocGen5USBPHYReset");
    do {
        sysUsDelay(10);
        tempRst = CSR_READ_4(pPhyDev, USB_DWC_GRSTCTL);

        if (++i > USB_PHY_RST_AHB_IDLE_TIMEOUT)
            {
            return;
            }
    } while ((tempRst & (UINT32)USB_DWC_RSTCTL_AHB_IDLE) == 0);
    BTT(">... AHB idle");

    /* Core Soft Reset */
    
    i = 0;
    tempRst |= USB_DWC_RSTCTL_SFT_RST;
    CSR_WRITE_4(pPhyDev, USB_DWC_GRSTCTL, tempRst);

    do {
        tempRst = CSR_READ_4(pPhyDev, USB_DWC_GRSTCTL);
        if (++i > USB_PHY_RST_TIMEOUT) 
            {
            break;
            }
        sysUsDelay(1);
    } while ((tempRst & USB_DWC_RSTCTL_SFT_RST) != 0);
    BTT(">... Soft RST");

    sysMsDelay(100);
    BTT("<usbAltSocGen5USBPHYReset");
    }

/*******************************************************************************
*
* usbAltSocGen5USBIntInit - initialize USB interrupt for alt_soc_gen5
*
* This routine initialize USB interrupt for alt_soc_gen5.
*
* RETURNS: N/A.
*
* ERRNO: N/A
*/

LOCAL void usbAltSocGen5USBIntInit
    (
    USB_PHY_DEV * pPhyDev
    )
    {
    UINT32 tempInitmsk = 0;

    /* Clear any pending OTG Interrupts */

    CSR_WRITE_4(pPhyDev, USB_DWC_GOTGINT, 0xFFFFFFFF);

    /* Clear any pending interrupts */

    CSR_WRITE_4(pPhyDev, USB_DWC_GINTSTS, 0xFFFFFFFF);

    /* Enable the interrupts in the GINTMSK. */

    tempInitmsk |= (UINT32)USB_DWC_INTMSK_MODE_MISMTC;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_OTG;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_RXFIFO_NOT_EMPT;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_USB_SUSP;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_CON_ID_STS_CHG;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_SES_DISCON_DET;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_NEW_SES_DET;
    tempInitmsk |= (UINT32)USB_DWC_INTMSK_WKP;

    CSR_WRITE_4(pPhyDev, USB_DWC_GINTMSK, tempInitmsk);
    }

/*******************************************************************************
*
* usbAltSocGen5USBPHYInit - initialize the USB PHY
*
* This routine initialize the USB PHY.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void usbAltSocGen5USBPHYInitInternal
    (
    USB_PHY_DEV * pPhyDev
    )
    {
    UINT32 usbHprt,hcfg;
    UINT32 ahbcfg = 0;
    UINT32 tempGusbcfg,usbcfg;
    static int usbInitDone = 0;

    BTT(">usbAltSocGen5USBPHYInitInternal");
    /* Common Initialization */

    tempGusbcfg = CSR_READ_4(pPhyDev, USB_DWC_GUSBCFG);

    /* Program the ULPI External VBUS bit if needed */

    if (pPhyDev->phyType == USB_PHY_TYPE_ULPI)
        {
        tempGusbcfg |= USB_DWC_USBCFG_ULPI_EXT_VBUS_DRV;
        }
    else
        {
        tempGusbcfg &= ~USB_DWC_USBCFG_ULPI_EXT_VBUS_DRV;
        }

    /* Set external TS Dline pulsing */
    
    tempGusbcfg = tempGusbcfg & (~((UINT32) USB_DWC_USBCFG_TERM_SEL_DL_PULSE));

    CSR_WRITE_4(pPhyDev, USB_DWC_GUSBCFG, tempGusbcfg);

    /* Reset USB Controller */
    
    usbAltSocGen5USBPHYReset(pPhyDev);

    /* Initialize parameters from Hardware configuration registers. */

    tempGusbcfg = CSR_READ_4(pPhyDev, USB_DWC_GUSBCFG);

    if (!usbInitDone) 
        {
        BTT("...!usbInitDone");
        tempGusbcfg = CSR_READ_4(pPhyDev, USB_DWC_GUSBCFG);
        usbInitDone = 1;

        if (tempGusbcfg & USB_DWC_USBCFG_ULPI_UTMI_SEL) 
            {
            /* ULPI interface */

            tempGusbcfg |= USB_DWC_USBCFG_PHYIF;

            /* 8bit data bus */

            tempGusbcfg &= ~((UINT32) USB_DWC_USBCFG_DDRSEL);  
            }
        
        CSR_WRITE_4(pPhyDev, USB_DWC_GUSBCFG, tempGusbcfg);

        /* Reset after setting the PHY parameters */
        
        usbAltSocGen5USBPHYReset(pPhyDev);
        }

    BTT("...ULPI");
    
    tempGusbcfg = CSR_READ_4(pPhyDev, USB_DWC_GUSBCFG);
    tempGusbcfg &= ~((UINT32) USB_DWC_USBCFG_ULPI_FSLS);
    tempGusbcfg &= ~((UINT32) USB_DWC_USBCFG_ULPI_CLK_SUS_M);
    CSR_WRITE_4(pPhyDev, USB_DWC_GUSBCFG, tempGusbcfg);

    /* Program the GAHBCFG Register. */
    
    ahbcfg = (ahbcfg & ~USB_DWC_AHBCFG_BURST_LEN(0xf)) | USB_DWC_AHBCFG_BURST_LEN(USB_DWC_GAHBCFG_INT_DMA_BURST_INCR);

    /* enable DMA */
    
    ahbcfg |= USB_DWC_AHBCFG_DMA_ENA;

    CSR_WRITE_4(pPhyDev, USB_DWC_GAHBCFG, ahbcfg);
    
    /* Program the GUSBCFG register. */
    
    tempGusbcfg = CSR_READ_4(pPhyDev, USB_DWC_GUSBCFG);

    tempGusbcfg |= USB_DWC_USBCFG_HNP_CAP;
    tempGusbcfg |= USB_DWC_USBCFG_SRP_CAP;
    CSR_WRITE_4(pPhyDev, USB_DWC_GUSBCFG, tempGusbcfg);

    /* Init USB interrupts */

    BTT(">usbAltSocGen5USBIntInit");
    usbAltSocGen5USBIntInit(pPhyDev);

    /* Disable the global interrupt   */
    
    ahbcfg = CSR_READ_4(pPhyDev, USB_DWC_GAHBCFG);
    ahbcfg = ahbcfg & (~USB_DWC_AHBCFG_GLBL_INT_MASK);
    CSR_WRITE_4(pPhyDev, USB_DWC_GAHBCFG, ahbcfg);
    
    /* Init USB HCD and set host mode */
    
    usbcfg = CSR_READ_4(pPhyDev, USB_DWC_GUSBCFG);
    usbcfg |= USB_DWC_USBCFG_FRC_HST_MODE;
    CSR_WRITE_4(pPhyDev, USB_DWC_GUSBCFG, usbcfg);

    /* Restart USB Phy Clock */
    
    CSR_WRITE_4(pPhyDev, USB_DWC_PCGCCTL, 0);

    /* Power USB PHY*/

    usbHprt = CSR_READ_4(pPhyDev, USB_DWC_HPRT);
    usbHprt = usbHprt & (~(0x1U << 2));
    usbHprt = usbHprt & (~(0x1U << 1));
    usbHprt = usbHprt & (~(0x1U << 3));
    usbHprt = usbHprt & (~(0x1U << 4));

    usbHprt = usbHprt | (0x1U << 12);
    
    CSR_WRITE_4(pPhyDev, USB_DWC_HPRT, usbHprt);

    /* Reset USB PHY */

    usbHprt = CSR_READ_4(pPhyDev, USB_DWC_HPRT);
    usbHprt = usbHprt & (~(0x1U << 2));
    usbHprt = usbHprt & (~(0x1U << 1));
    usbHprt = usbHprt & (~(0x1U << 3));
    usbHprt = usbHprt & (~(0x1U << 4));

    usbHprt = usbHprt | (0x1U << 8);
    
    CSR_WRITE_4(pPhyDev, USB_DWC_HPRT, usbHprt);

    sysMsDelay(50);
    BTT("... after reset delay");

    /* Release PHY from reset */

    usbHprt = CSR_READ_4(pPhyDev, USB_DWC_HPRT);
    usbHprt = usbHprt & (~(0x1U << 2));
    usbHprt = usbHprt & (~(0x1U << 1));
    usbHprt = usbHprt & (~(0x1U << 3));
    usbHprt = usbHprt & (~(0x1U << 4));

    usbHprt = usbHprt & (~(0x1U << 8));
    
    CSR_WRITE_4(pPhyDev, USB_DWC_HPRT, usbHprt);
    sysMsDelay(1000);
    BTT("... after 1s delay");

    /* High speed PHY running at full speed or high speed USB_DWC_HCFG_30_60_MHZ */

    hcfg = CSR_READ_4(pPhyDev, USB_DWC_HCFG);
    hcfg = hcfg & (~(0x03U));
    CSR_WRITE_4(pPhyDev, USB_DWC_HCFG, hcfg);
    BTT("<usbAltSocGen5USBPHYInitInternal");

    return;
    }
