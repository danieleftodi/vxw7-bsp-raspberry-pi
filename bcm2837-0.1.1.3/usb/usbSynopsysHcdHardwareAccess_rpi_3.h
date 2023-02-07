/* usbSynopsysHcdHardwareAccess_rpi_3.h - hardware access routines for Synopsys
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

This contains some basic routines which handle the Synopsys USB
host controller hardware.

*/

#ifndef __INCSynopsysHcdHardwareAccess_rpi_3h
#define __INCSynopsysHcdHardwareAccess_rpi_3h

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#else
#    define SWAP32 
#endif /* ARMBE8 */


/* This macro writes the 32-bit value to the reigiter */

#if CPU==MIPSI64R2
#define USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData, offset, value)            \
    pSynopsysHcdData->pRegWrite32Handle((pSynopsysHcdData->regBase + offset),   \
                                        value)
#else
#define USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData, offset, value)            \
    vxbWrite32(((pUSB_SYNOPSYSHCD_DATA)(pSynopsysHcdData))->pRegWrite32Handle,  \
        (void *)(((pUSB_SYNOPSYSHCD_DATA)(pSynopsysHcdData))->regBase + offset),\
        SWAP32((UINT32) value))
#endif

/* This macro gets the 32-bit value from the reigiter */

#if CPU==MIPSI64R2
#define USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData, offset)                    \
    pSynopsysHcdData->pRegRead32Handle(pSynopsysHcdData->regBase + offset)
#else
#define USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData, offset)                        \
    SWAP32(vxbRead32(((pUSB_SYNOPSYSHCD_DATA)(pSynopsysHcdData))->pRegRead32Handle, \
         (void *)(((pUSB_SYNOPSYSHCD_DATA)(pSynopsysHcdData))->regBase + offset))) 
#endif

#if CPU==MIPSI64R2
/* This macro writes the 64-bit value to the reigiter */

#define USB_SYNOPSYSHCD_WRITE64_REG(pSynopsysHcdData, offset, value)           \
    pSynopsysHcdData->pRegWrite64Uint64Handle(                                 \
                                        (pSynopsysHcdData->regBase + offset),  \
                                        value)

/* This macro gets the 64-bit value from the reigiter */

#define USB_SYNOPSYSHCD_READ64_REG(pSynopsysHcdData, offset)                   \
    pSynopsysHcdData->pRegRead64Uint64Handle(                                  \
                                         pSynopsysHcdData->regBase + offset)
#endif

/* This macro uses mask code to mask the related register value */

#define USB_SYNOPSYSHCD_SETBITS32_REG(pSynopsysHcdData, offset, mask)          \
    {                                                                          \
    UINT32 value = 0x00;                                                       \
    value = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData, offset);              \
    value = value | mask;                                                      \
    USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData, offset, value);              \
    }

/* This macro uses mask code to un-mask the related register value */

#define USB_SYNOPSYSHCD_CLEARBITS32_REG(pSynopsysHcdData, offset, mask)        \
    {                                                                          \
    UINT32 value = 0x00;                                                       \
    value = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData, offset);              \
    value = value & (~mask);                                                   \
    USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData, offset, value);              \
    }

#define USB_SYNOPSYSHCD_SETBITS32_HPRT_REG(pSynopsysHcdData, offset, mask)     \
    {                                                                          \
    UINT32 value = 0x00;                                                       \
    value = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData, offset);              \
    value = value & (UINT32)(~(0x3f));                                         \
    value = value | mask;                                                      \
    USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData, offset, value);              \
    }

/* This macro uses mask code to un-mask the related register value */

#define USB_SYNOPSYSHCD_CLEARBITS32_HPRT_REG(pSynopsysHcdData, offset, mask)   \
    {                                                                          \
    UINT32 value = 0x00;                                                       \
    value = USB_SYNOPSYSHCD_READ32_REG(pSynopsysHcdData, offset);              \
    value = value & (UINT32)(~(0x3f));                                         \
    value = value & (~mask);                                                   \
    USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData, offset, value);              \
    }

/* This macro programes the DMA address */

#if CPU==MIPSI64R2
#define USB_SYNOPSYSHCD_PROGRAM_DMA_ADDRESS(pSynopsysHcdData,offset,address)   \
    {                                                                          \
    UINT64 dmaAddr = 0;                                                        \
    dmaAddr = (pSynopsysHcdData->regBase + offset) & ~(0x10000000);            \
    pSynopsysHcdData->pRegWrite64Uint64Handle(dmaAddr, address);               \
    pSynopsysHcdData->pRegRead64Uint64Handle(dmaAddr);                         \
    }
#else
#define USB_SYNOPSYSHCD_PROGRAM_DMA_ADDRESS(pSynopsysHcdData,offset,address)   \
    {                                                                          \
    USB_SYNOPSYSHCD_WRITE32_REG(pSynopsysHcdData,offset,address);              \
    }
#endif

VOID usbSynopsysHcdFlushTxFIFO
    (
    pUSB_SYNOPSYSHCD_DATA pSynopsysHcdData,
    int TxFIFONum
    );
VOID usbSynopsysHcdFlushRxFIFO
    (
    pUSB_SYNOPSYSHCD_DATA pSynopsysHcdData
    );
VOID usbSynopsysHCDCoreInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );
VOID usbSynopsysHCDHostInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );

void usbSynopsysHCDResetCore
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    );
    
#ifdef __cplusplus
}
#endif

#endif /* __INCSynopsysHcdHardwareAccess_rpi_3h*/

/* End of file */
