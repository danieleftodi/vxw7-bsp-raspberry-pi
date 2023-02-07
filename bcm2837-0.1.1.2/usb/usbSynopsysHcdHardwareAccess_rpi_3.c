/* usbSynopsysHcdHardwareAccess_rpi_3.c - hardware access routines for Synopsys
 * HCD
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

This file contains the hardware access entries provided by the
Synopsys USB Host Controller Driver.

INCLUDE FILES: usbSynopsysHcdRegisterInfo_rpi_3.h,
               usbSynopsysHcdDataStructures_rpi_3.h,
               usbSynopsysHcdHardwareAccess_rpi_3.h
*/

/* includes */

#include "usbSynopsysHcdRegisterInfo_rpi_3.h"
#include "usbSynopsysHcdDataStructures_rpi_3.h"
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"

#define USB_SYNOPSYSHCD_FIFO_ALIGN_MASK                     (0xFFFFFFF8)

/* forwards*/

UINT32 usbSynopsysDwcHcdReadHptr
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );
    
/*******************************************************************************
*
* usbSynopsysHcdFlushTxFIFO - flush the related TX FIFO
*
* This routine flushs the <TxFIFONum> TX FIFO.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdFlushTxFIFO
    (
    pUSB_SYNOPSYSHCD_DATA pSynopsysHcdData, /* Pointer to HCD block */
    int                   TxFIFONum         /* TX FIFO number to be flushed */
    )
    {
    volatile UINT32 regValue = 0;
    int uTimeOutCount = 10000;

    /* Wait AHB idle  */

    for(;;)
        {
        regValue = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData,
                                              USB_SYNOPSYSHCD_GRSTCTL);
        uTimeOutCount--;
        if ((regValue & USB_SYNOPSYSHCD_GRSTCTL_AHBIDLE)||(uTimeOutCount < 0))
            break;
        }

    /* Flush all periodic/nonperiodic FIFO in usb core */

    regValue = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData,
                                          USB_SYNOPSYSHCD_GRSTCTL);

    /* Set the TxFIFONum */

    regValue &= ~USB_SYNOPSYSHCD_GRSTCTL_TXFNUM;
    regValue |= (UINT32)((TxFIFONum & 0x1F) << USB_SYNOPSYSHCD_GRSTCTL_TXFNUM_OFFSET);

    /* Set the TXFFLSH bit */

    regValue |= USB_SYNOPSYSHCD_GRSTCTL_TXFFLSH;

    USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData,
                                USB_SYNOPSYSHCD_GRSTCTL,
                                regValue);

    /* Wait until the TXFFLSH to be cleard by usb core*/
    /* The Datasheet said it will take 8 clock cycle*/

    do
        {
        regValue = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData,
                                              USB_SYNOPSYSHCD_GRSTCTL);
        }while(0 != (regValue & USB_SYNOPSYSHCD_GRSTCTL_TXFFLSH));

    return ;
    }

/*******************************************************************************
*
* usbSynopsysHcdFlushRxFIFO - flush the RX FIFO
*
* This routine flushs the RX FIFO.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdFlushRxFIFO
    (
    pUSB_SYNOPSYSHCD_DATA pSynopsysHcdData /* Pointer to HCD block */
    )
    {
    volatile UINT32 regValue = 0;
    int uTimeOutCount = 10000;

    /* Wait AHB idle  */

    for(;;)
        {
        regValue = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData,
                                              USB_SYNOPSYSHCD_GRSTCTL);
        uTimeOutCount--;
        if ((regValue & USB_SYNOPSYSHCD_GRSTCTL_AHBIDLE)||(uTimeOutCount < 0))
            break;
        }

    /* Set the RXFFLSH bit */

    USB_SYNOPSYSHCD_SETBITS32_REG(pSynopsysHcdData,
                                  USB_SYNOPSYSHCD_GRSTCTL,
                                  USB_SYNOPSYSHCD_GRSTCTL_RXFFLSH);

    /* Wait until the RXFFLSH to be cleard by usb core*/
    /* The Datasheet said it will take 8 clock cycle*/

    do
        {
        regValue = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData,
                                              USB_SYNOPSYSHCD_GRSTCTL);
        }while(0 != (regValue & USB_SYNOPSYSHCD_GRSTCTL_RXFFLSH));

    return ;
    }

/*******************************************************************************
*
* usbSynopsysHCDResetCore - soft reset the USB Core
*
* This routine does soft reset to the USB Core to reinit the statemachine.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHCDResetCore
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    volatile UINT32 uGrstctlReg = 0x0;
    int uTimeOutCount = 10000;

    /* Wait AHB idle */

    for(;;)
        {
        uGrstctlReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                 USB_SYNOPSYSHCD_GRSTCTL);
        uTimeOutCount--;
        if ((uGrstctlReg & USB_SYNOPSYSHCD_GRSTCTL_AHBIDLE)||(uTimeOutCount < 0))
            break;
        }

    /* Soft reset USB core */

    uGrstctlReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_GRSTCTL);
    uGrstctlReg |= USB_SYNOPSYSHCD_GRSTCTL_CSFTRST;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_GRSTCTL, uGrstctlReg);

    /* Wait reset finish */

    do
        {
        (void) OS_DELAY_MS(1);
        uGrstctlReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                 USB_SYNOPSYSHCD_GRSTCTL);
        }while(0 != (uGrstctlReg & USB_SYNOPSYSHCD_GRSTCTL_CSFTRST));

    /* After reset we must wait at lease 3 phy clocks */

    (void)OS_DELAY_MS(100);

}

/*******************************************************************************
*
* usbSynopsysHCDCoreInit - initialize the USB core
*
* This routine initializes the USB Core.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHCDCoreInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    UINT32 uGahbcfgReg = 0x0;
    UINT32 uGusbcfgReg = 0x0;

    if(NULL == pHCDData)
        return;

    /* USB Core Initialization */

    /* Reset USB Core Statemachine */

    usbSynopsysHCDResetCore(pHCDData);

    /* Configure the GAHBCFG register */

    uGahbcfgReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_GAHBCFG);

    /* Configure the transfer mode */

    /* Now we always use DMA */

    uGahbcfgReg |= USB_SYNOPSYSHCD_GAHBCFG_DMAEN;
    uGahbcfgReg &= ~USB_SYNOPSYSHCD_GAHBCFG_HBSTLEN;
    uGahbcfgReg |= USB_SYNOPSYSHCD_GAHBCFG_NPTXFEMPLVL;
    uGahbcfgReg |= USB_SYNOPSYSHCD_GAHBCFG_PTXFEMPLVL;
    uGahbcfgReg |= USB_SYNOPSYSHCD_GAHBCFG_GLBLINTRMSK;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_GAHBCFG, uGahbcfgReg);

    /* Configure the GUSBCFG Register */

    uGusbcfgReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_GUSBCFG);
    uGusbcfgReg &= ~USB_SYNOPSYSHCD_GUSBCFG_TOUTCAL;
    uGusbcfgReg &= ~USB_SYNOPSYSHCD_GUSBCFG_DDRSEL;
    uGusbcfgReg &= ~USB_SYNOPSYSHCD_GUSBCFG_USBTRDTIM;
    uGusbcfgReg |=  (0x05 << USB_SYNOPSYSHCD_GUSBCFG_USBTRDTIM_OFFSET);
    uGusbcfgReg &= ~USB_SYNOPSYSHCD_GUSBCFG_PHYLPWRCLKSEL;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_GUSBCFG, uGusbcfgReg);

}

/*******************************************************************************
*
* usbSynopsysHCDHostInit - initialize the USB to work in host mode
*
* This routine initializes the USB to work in host mode.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHCDHostInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    UINT32 uTempRegValue = 0x0;
    UINT32 uGrxfsizReg = 0x0;
    UINT32 uGnptxfsizReg = 0x0;
    UINT32 uHptxfsizReg = 0x0;
    UINT32 uDFIFOSize = 0x0;
    UINT32 uRxFIFOSize = 0x0;
    UINT32 uNpTxFIFOSize = 0x0;
    UINT32 uPTxFIFOSize = 0x0;
    int    i = 0;

    if(NULL == pHCDData)
        return;

    /* Init the host side function */

    /* Reset the PHY Clock */

    uTempRegValue = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_PCGCCTL);
    uTempRegValue &= ~USB_SYNOPSYSHCD_PCGCCTL_PHYSUSPENDED;
    uTempRegValue &= ~USB_SYNOPSYSHCD_PCGCCTL_RSTPDWNMODULE;
    uTempRegValue &= ~USB_SYNOPSYSHCD_PCGCCTL_PWRCLMP;
    uTempRegValue &= ~USB_SYNOPSYSHCD_PCGCCTL_GATEHCLK;
    uTempRegValue &= ~USB_SYNOPSYSHCD_PCGCCTL_STOPPCLK;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_PCGCCTL,
                                uTempRegValue);

    /*
     * Program the USBC_HCFG register to select full-speed host or
     * high-speed host.
     */

    uTempRegValue = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HCFG);

    /* Support HS/FS/LS speed */

    uTempRegValue &= ~ USB_SYNOPSYSHCD_HCFG_FSLSSUPP;

    /* PHY Clock run at 30/60 MHZ */

    uTempRegValue &= ~ USB_SYNOPSYSHCD_HCFG_FSLPCLKSEL;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HCFG, uTempRegValue);

    /* Set the RX/TX FIFO Size */

    /* Get the total DFIFO size */

    uTempRegValue = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_GHWCFG3);
    uDFIFOSize = (uTempRegValue & USB_SYNOPSYSHCD_GHWCFG3_DFIFODEPTH) >>
                 USB_SYNOPSYSHCD_GHWCFG3_DFIFODEPTH_OFFSET;

    /* Program the RX FIFO size */

    uGrxfsizReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_GRXFSIZ);

    uRxFIFOSize = ((uDFIFOSize * 30) / 100) & USB_SYNOPSYSHCD_FIFO_ALIGN_MASK;

    uGrxfsizReg = (uRxFIFOSize << USB_SYNOPSYSHCD_GRXFSIZ_RXFDEP_OFFSET) & \
                  USB_SYNOPSYSHCD_GRXFSIZ_RXFDEP;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_GRXFSIZ, uGrxfsizReg);

    /* Program the non-periodic TX FIFO size */

    uNpTxFIFOSize = ((uDFIFOSize * 50) / 100) & USB_SYNOPSYSHCD_FIFO_ALIGN_MASK;
    uGnptxfsizReg = ((uNpTxFIFOSize << USB_SYNOPSYSHCD_GNPTXFSIZ_NPTXFDEP_OFFSET) & \
                     USB_SYNOPSYSHCD_GNPTXFSIZ_NPTXFDEP) | \
                    (uRxFIFOSize & USB_SYNOPSYSHCD_GNPTXFSIZ_NPTXFSTADDR);
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_GNPTXFSIZ, uGnptxfsizReg);

    /* Program the periodic TX FIFO size */

    uPTxFIFOSize = uDFIFOSize - uRxFIFOSize - uNpTxFIFOSize;
    uHptxfsizReg = ((uPTxFIFOSize << USB_SYNOPSYSHCD_HPTXFSIZ_PTXFSIZE_OFFSET) &
                     USB_SYNOPSYSHCD_HPTXFSIZ_PTXFSIZE) |
                    ((uRxFIFOSize + uNpTxFIFOSize) &
                      USB_SYNOPSYSHCD_HPTXFSIZ_PTXFSTADDR);
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HPTXFSIZ,
                                uHptxfsizReg);

    /* Flush all FIFOs */

    usbSynopsysHcdFlushTxFIFO(pHCDData, 0x10);
    usbSynopsysHcdFlushRxFIFO(pHCDData);

    /*
     * Flush all channels and leftover queue to put every
     * channel into known state
     */

    for (i = 0 ; i < pHCDData->hostNumDmaChannels; i++)
        {
        uTempRegValue = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                   USB_SYNOPSYSHCD_HCCHAR(i));
        uTempRegValue &= ~USB_SYNOPSYSHCD_HCCHAR_CHENA;
        uTempRegValue |= USB_SYNOPSYSHCD_HCCHAR_CHDIS;
        uTempRegValue &= ~USB_SYNOPSYSHCD_HCCHAR_EPDIR;
        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HCCHAR(i),
                                    uTempRegValue);
        }

    for(i = 0 ; i < pHCDData->hostNumDmaChannels; i++)
        {
        uTempRegValue = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                   USB_SYNOPSYSHCD_HCCHAR(i));
        uTempRegValue |= USB_SYNOPSYSHCD_HCCHAR_CHENA;
        uTempRegValue |= USB_SYNOPSYSHCD_HCCHAR_CHDIS;
        uTempRegValue &= ~USB_SYNOPSYSHCD_HCCHAR_EPDIR;
        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                    USB_SYNOPSYSHCD_HCCHAR(i),
                                    uTempRegValue);
        do
            {
            (void) OS_DELAY_MS(1);
            uTempRegValue = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                       USB_SYNOPSYSHCD_HCCHAR(i));
            }while(0 != (uTempRegValue & USB_SYNOPSYSHCD_HCCHAR_CHENA));
        }


    /* Program the port power bit to drive VBUS on the USB */

    uTempRegValue = usbSynopsysDwcHcdReadHptr(pHCDData);
    uTempRegValue |= USB_SYNOPSYSHCD_HPRT_PRTPWR;
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT, uTempRegValue);
}

/*******************************************************************************
*
* usbSynopsysDwcHcdReadHptr - read HPRT bits except one which are write-one-clear
*
* This routine iRead HPRT bits except one which are write-one-clear.
*
* RETURNS: value
*
* ERRNO: N/A
*
* \NOMANUAL
*/

UINT32 usbSynopsysDwcHcdReadHptr
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    )
    {
    UINT32 value;

    value = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT);
    value &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
    value &= ~USB_SYNOPSYSHCD_HPRT_PRTCONNDET;
    value &= ~USB_SYNOPSYSHCD_HPRT_PRTENCHNG;
    value &= ~USB_SYNOPSYSHCD_HPRT_PRTOVRCURRCHNG;

    return value;
    }

/* End of file */
