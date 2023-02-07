/* usbSynopsysHcdEventHandler_rpi_3.c - Synopsys USB HCD interrupt handler
 * module 
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

This file contains interrupt handling routines which handle the Synopsys
USB host controller interrupts.

INCLUDE FILES:  vxWorks.h, cacheLib.h, usb/usb.h
                usbSynopsysHcdEventHandler_rpi_3.h,
                usbSynopsysHcdHardwareAccess_rpi_3.h,
                usbSynopsysHcdDataStructures_rpi_3.h,
                usbSynopsysHcdInterfaces_rpi_3.h,
                usbSynopsysHcdRegisterInfo_rpi_3.h,
                usbSynopsysHcdRhEmulation_rpi_3.h,
                usbSynopsysHcdUtil_rpi_3.h
*/

#include <vxWorks.h>
#include <cacheLib.h>
#include <private/vmLibP.h>
#include <usb.h>
#include "usbSynopsysHcdEventHandler_rpi_3.h"
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"
#include "usbSynopsysHcdDataStructures_rpi_3.h"
#include "usbSynopsysHcdInterfaces_rpi_3.h"
#include "usbSynopsysHcdRegisterInfo_rpi_3.h"
#include "usbSynopsysHcdRhEmulation_rpi_3.h"
#include "usbSynopsysHcdUtil_rpi_3.h"

/* defines */

#define USB_SYNOPSYSHCD_REQUEST_STAGE_INIT_MASK  (0xFFFFFFFE)

/*
 * The hardware will not update the HAINT register
 * although the HCINT has channel interrupt status.
 * We use the <USB_SYNOPSYSHCD_HCINT_MASK> to catch the status we have tested.
 */

#if defined(_WRS_CONFIG_ALTERA_SOC_STRATIX10_VP)
#define USB_SYNOPSYSHCD_HCINT_MASK      (USB_SYNOPSYSHCD_HCINT_ACK  | \
                                         USB_SYNOPSYSHCD_HCINT_NYET | \
                                         USB_SYNOPSYSHCD_HCINT_NAK  | \
                                         USB_SYNOPSYSHCD_HCINT_FRMOVRUN | \
                                         USB_SYNOPSYSHCD_HCINT_XFERCOMPL)
#else
#define USB_SYNOPSYSHCD_HCINT_MASK      (USB_SYNOPSYSHCD_HCINT_ACK  | \
                                         USB_SYNOPSYSHCD_HCINT_NYET | \
                                         USB_SYNOPSYSHCD_HCINT_NAK  | \
                                         USB_SYNOPSYSHCD_HCINT_FRMOVRUN)
#endif /* _WRS_CONFIG_ALTERA_SOC_STRATIX10_VP */

/* globals */

IMPORT pUSB_SYNOPSYSHCD_DATA * g_pSynopsysHCDData;

/*
 * This is actually a dynamically allocated array
 * of size USB_MAX_SYNOPSYSHCI_COUNT.
 */

IMPORT spinlockIsr_t spinLockIsrSynopsysHcd[];

/* forward declaration */

VOID usbSynopsysHcdCompleteProcess
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    USBHST_STATUS                 uCompleteStatus
    );
BOOLEAN usbSynopsysHcdCopyRHInterruptData
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    UINT32      uStatusChange
    );
LOCAL VOID usbSynopsysHcdChannelStartProcess
    (
    pUSB_SYNOPSYSHCD_DATA  pHCDData,
    pUSB_SYNOPSYSHCD_PIPE  pHCDPipe,
    UINT8                  uChannel
    );
LOCAL __inline__ VOID usbSynopsysHcdFillTaskInfoAndSend
    (
    pUSB_SYNOPSYSHCD_DATA          pHCDData,
    pUSB_SYNOPSYSHCD_PIPE          pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO  pRequestInfo,
    UINT32                         uChannel,
    UINT32                         uCmdCode,
    USBHST_STATUS                  uCompleteStatus
    );

LOCAL VOID usbSynopsysHcdPortChangeHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );

#if defined(_WRS_CONFIG_ALTERA_SOC_STRATIX10_VP)
LOCAL VOID usbSynopsysDwcHcdChannelProcessXFERCOMPLResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel,
    UINT32                        uLengthLeftToTransfer,
    UINT32                        uLengthThisTransfer
    );
#endif
LOCAL VOID usbSynopsysHcdChannelProcessACKResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel,
    UINT32                        uLengthLeftToTransfer,
    UINT32                        uLengthThisTransfer
    );

LOCAL VOID usbSynopsysHcdChannelProcessNAKResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo
    );

LOCAL VOID usbSynopsysHcdChannelProcessXACTERRResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel
    );

LOCAL VOID usbSynopsysHcdChannelProcessDATAGLERResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel
    );

LOCAL VOID usbSynopsysHcdChannelProcessFRMOVRUNResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel
    );

LOCAL VOID usbSynopsysHcdChannelInterruptHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    UINT8                 uChannel,
    UINT32                uUsbHcintReg
    );

LOCAL BOOL usbSynopsysHcdPipeNeedSchedule
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe
    );

LOCAL pUSB_SYNOPSYSHCD_PIPE usbSynopsysHcdCheckSchedule
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe
    );

LOCAL VOID usbSynopsysHcdControlChannelStart
    (
    pUSB_SYNOPSYSHCD_DATA   pHCDData,
    pUSB_SYNOPSYSHCD_PIPE   pHCDPipe,
    UINT8                   uChannel,
    UINT32                  uLengthToTransfer
    );

LOCAL __inline__ VOID usbSynopsysProgramHCCHAR
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel
    );

LOCAL __inline__ VOID usbSynopsysProgramChannelInterrupt
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel
    );

LOCAL __inline__ VOID usbSynopsysProgramHCTSIZ
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT8                         uChannel,
    UINT32                        uLengthToTransfer
    );

LOCAL PHYS_ADDR usbSynopsysHcdVirToPhy
    (
    VIRT_ADDR virtAdrs
    );

/*******************************************************************************
*
* usbShdrcHsDmaFreeChannelRequest - request one free channel of the DMA
*
* This routine is to request one free channel of the DMA.
*
* RETURNS: Free DMA channel index, or ERROR if no free channel is found
*
* ERRNO: N/A
*
* \NOMANUAL
*/

int usbShdrcHsDmaFreeChannelRequest
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    )
    {
    int index;

    if (pHCDData == NULL)
        {
        USB_SHCD_WARN("usbShdrcHsDmaFreeChannelRequest "
                      "failed pHCDData is NULL.\n", 0,0, 0, 0, 0, 0);

        return USB_SHDRC_DMA_INVALID_CHANNEL;
        }

    (void)semTake(pHCDData->dmaChannelMutex, WAIT_FOREVER);

    index = USB_SYNOPSYSHCD_GET_FIRST_IDLE_DMACHANNEL(pHCDData->uIdleDmaChannelMap);

    /* Mark the channel as busy */

    if (index < pHCDData->hostNumDmaChannels)
        {
        pHCDData->uIdleDmaChannelMap = (UINT32)(pHCDData->uIdleDmaChannelMap &
                                                   ~(0x01 << index));
        }
    else
        {
        USB_SHCD_WARN("usbShdrcHsDmaFreeChannelRequest failed %d, %d.\n",
                      index, pHCDData->hostNumDmaChannels, 0, 0, 0, 0);
        index = USB_SHDRC_DMA_INVALID_CHANNEL;
        }

    (void)semGive(pHCDData->dmaChannelMutex);

    return index;
    }

/*******************************************************************************
*
* usbShdrcHsDmaChannelRelease - release a busy DMA channel
*
* This routine is used to release a busy DMA channel.
*
* RETURNS: OK, or ERROR if failed
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbShdrcHsDmaChannelRelease
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    int                   uChannel
    )
    {
    if ((pHCDData == NULL) || (uChannel > pHCDData->hostNumDmaChannels))
        return ERROR;

    (void)semTake(pHCDData->dmaChannelMutex, WAIT_FOREVER);

    pHCDData->uIdleDmaChannelMap |= (UINT32)((0x01 << uChannel));

    (void)semGive(pHCDData->dmaChannelMutex);

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdISR - interrupt service routine for the Synopsys HCD driver
*
* This routine is the interrupt service routine for the Synopsys HCD driver.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

VOID usbSynopsysHcdISR
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    /* To store the fields of interrupt status register */

    UINT32 uInterruptStatus = 0;

    /* To store the fields of interrupt enable register */

    UINT32 uInterruptMask = 0;

    /* To store the fields of HAINT register */

    UINT32 uHaintReg = 0x0;

    /* Common variable to store the fields of USB registers */

    UINT32 uTempRegStatus = 0;

    UINT8 i;

    /* Check the validity of the parameter */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Parameter is not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    if (pHCDData->isrMagic != USB_SYNOPSYSHCD_MAGIC_ALIVE)
        {
        USB_SHCD_ERR("ISR not active, maybe shared?\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

    /* Get the interrupt mask */

    uInterruptMask = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                USB_SYNOPSYSHCD_GINTMSK);
    /* Get the interrupt status */

    uInterruptStatus = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_GINTSTS);

    /* Mask all not enabled interrupts */

    uInterruptStatus = (uInterruptMask & uInterruptStatus);

    /* Store the interrupt status */

    if (uInterruptStatus)
        {
        (void) vxAtomicOr((atomic_t *)&pHCDData->uInterruptStatus,
                          (atomicVal_t)uInterruptStatus);
        }

    /* With port Change interrupt, we need clear the subordinate status */

    if ((uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_DISCONNINT) ||
        (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_PRTINT))
        {
        uTempRegStatus = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                    USB_SYNOPSYSHCD_HPRT);

        /* Store the port status */

        (void) vxAtomicSet((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)uTempRegStatus);

        /*
         * Clear the port change bit, port enable bit is R/W1C so we should
         * aware not clear the port enable bit
         */

        uTempRegStatus &= ~(USB_SYNOPSYSHCD_HPRT_PRTENA);
        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                    USB_SYNOPSYSHCD_HPRT,
                                    uTempRegStatus);
        }

    /* With Channel Related interrupt,we need clear the subordinate status */

    if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_HCHINT)
        {

        /* Read the HAINT */

        uHaintReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_HAINT);
        /* Store the HAINT status */

        (void) vxAtomicOr((atomic_t *)&pHCDData->uHAINTStatus,
                          (atomicVal_t)uHaintReg);

        /* Read the HCINTn and clear */

        /*
         * Although there are some channels not enabled,
         * we also should clean the interrupt
         */

        for (i = 0; i < pHCDData->hostNumDmaChannels; i++)
            {
            uTempRegStatus = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                        USB_SYNOPSYSHCD_HCINT(i));
            /* Store the HCINT status */

            (void) vxAtomicOr((atomic_t *)&pHCDData->uHCINTnStatus[i],
                              (atomicVal_t)uTempRegStatus);

            /*
             * NOTE: The following code is for hardware workaround.
             *
             * When we transfer data, it seems we cann't get the response
             * from the hardware sometimes, this causes the related request
             * timeout.
             *
             * From the analyzer the hardware does ACK the transfer
             * and from the HCINT register we do have the ACK and CHHLTD
             * status. But we don't have the related channel interrupt,
             * so we cann't process the channel interrupt to take the next action.
             *
             * When we test the split bulk transfer we find that the NYET and CHHLTD
             * in HCINT register but we still cann't get the HAINT interrupt.
             *
             * When we test the hub+usb flash disk read/write sometimes detach
             * the usb flash disk then attach it on another port the hub cann't
             * detect the device. This is because the hub's periodic request
             * is not handled properly. The HCINT register has status
             * FRMORUN and CHHLTD but the HAINT register doesn't trigger any
             * interrupt for this channel.
             *
             * When we frequently attach/detach the hub + usb flash disk,
             * The HCINT register has staus NAK and CHHLTD. The HAINT register
             * may still not have any bit to indicate the interrupt.
             *
             * To workaround the above situation, if the HCINT has any status
             * (ACK/NAK/NYET/FRMORUN) and CHHLTD, but the HAINT doesn't have any
             * interrupt set for this channel, we will set the HAINT manually.
             */

            if ((0 == (uHaintReg & (0x01 << i ))) &&
                ((uTempRegStatus & USB_SYNOPSYSHCD_HCINT_MASK) != 0x0) &&
                ((uTempRegStatus & USB_SYNOPSYSHCD_HCINT_CHHLTD) != 0x0))
                {
                (void) vxAtomicOr((atomic_t *) &pHCDData->uHAINTStatus,
                                  ((atomicVal_t) 0x01 << i));
                }
            /* End of workaround */

            /* Clear the HCINT interrupt */

            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData ,
                                        USB_SYNOPSYSHCD_HCINT(i),
                                        uTempRegStatus);
            }

        }

    /* Clear some of the interupt status(some can not clear in this step) */

    USB_SYNOPSYSHCD_WRITE32_REG (pHCDData ,
                                 USB_SYNOPSYSHCD_GINTSTS,
                                 uInterruptStatus);

    SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

    /* Release the event to server the interrupt */

    if (uInterruptStatus)
        {
        /* Send message to the high priority thread */

#ifdef USB_SHCD_POLLING_MODE
        USB_SHCD_VDBG("send interrutpt status %x \n",uInterruptStatus,
                      2, 3, 4, 5, 6);
#endif
        (void)OS_RELEASE_EVENT(pHCDData->interruptEvent);
        }
    return;
    }

/*******************************************************************************
*
* usbShcdPollingISR - polling thread entry for synopsys HCD
*
* This routine is the pollling thread entry for synopsys HCD.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbShcdPollingISR
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    )
    {
    while (1)
        {
        /* Call the ISR function */

        usbSynopsysHcdISR(pHCDData);

        (void)taskDelay(USB_SHCD_POLLING_INTERVAL);
        }
    }

/*******************************************************************************
*
* usbSynopsysHcdInterruptHandler - handle Synopsys HCD interrupts
*
* This routine handles the Synopsys HCD interrupts.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

VOID usbSynopsysHcdInterruptHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    UINT32 uInterruptStatus = 0;
    UINT8  uChannel = 32;
    UINT32 uUsbGinmsk = 0x0;
    UINT32 uUsbHaintReg = 0x0;
    UINT32 uUsbHcintReg = 0x0;

    /* Check the validity of the parameter */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);

        return;
        }

    while (TRUE)
        {

        /* Wait on the signalling of the event */

        (void)OS_WAIT_FOR_EVENT(pHCDData->interruptEvent, WAIT_FOREVER);

        /* Read the contents of the interrupt status */

        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        uInterruptStatus = (UINT32)pHCDData->uInterruptStatus;
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

        /* If there are no interrupts observed, continue */

        if (0 == (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_INTERRUPT_MASK))
            {
            continue;
            }

        /*
         * Note about how to clear saved interrupt status bits:
         *
         * We have already recorded the interrupt status at local variable
         * <uInterruptStatus>. So we'd better clear the related interrupt
         * status before we enter the related interrupt handler.
         * The controller may raise the same interrupt during the time we are
         * processing it (thus the interrupt bit will need to be set again),
         * if we clear the interrupt bit after the handler is called, we
         * may incorrectly clear the status bit (the 2nd trigger that happens
         * during the processing), and thus may cause us to lose some interrupt.
         */

        /* The "WKUPINT" interrupt is not supported only unmask it */

        if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_WKUPINT)
            {
            SPIN_LOCK_ISR_TAKE (&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            vxAtomicAnd((atomic_t *)&pHCDData->uInterruptStatus,
                        (atomicVal_t)~USB_SYNOPSYSHCD_GINTSTS_WKUPINT );
            SPIN_LOCK_ISR_GIVE (&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            }

        /* The "SESSREQINT" interrupt is not supported only unmask it */

        if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_SESSREQINT)
            {
            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            vxAtomicAnd((atomic_t *)&pHCDData->uInterruptStatus,
                        (atomicVal_t)~USB_SYNOPSYSHCD_GINTSTS_SESSREQINT );
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            }

        /* Port disconnect interrupt */

        if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_DISCONNINT)
            {
            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            vxAtomicAnd((atomic_t *)&pHCDData->uInterruptStatus,
                        (atomicVal_t)~USB_SYNOPSYSHCD_GINTSTS_DISCONNINT);
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            usbSynopsysHcdPortChangeHandler(pHCDData);
            }

        /* Port connect interrupt */

        if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_PRTINT)
            {
            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            vxAtomicAnd((atomic_t *)&pHCDData->uInterruptStatus,
                        (atomicVal_t)~USB_SYNOPSYSHCD_GINTSTS_PRTINT);
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            usbSynopsysHcdPortChangeHandler(pHCDData);
            }

        /* Host channel Interrupt */

        if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_HCHINT)
            {
            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            /* Clear the channel interrupt */

            vxAtomicAnd((atomic_t *)&pHCDData->uInterruptStatus,
                        (atomicVal_t)(~USB_SYNOPSYSHCD_GINTSTS_HCHINT));

            /* Read the HAINT register see which channel has the interrupt */

            uUsbHaintReg = (UINT32)pHCDData->uHAINTStatus;
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            while (uUsbHaintReg)
                {
                uChannel = (UINT8)USB_SYNOPSYSHCD_FIND_FIRST_SET_BIT(uUsbHaintReg);

                /* The channel can not bigger than the max channel count */

                if (uChannel > pHCDData->hostNumDmaChannels)
                    break;

                SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

                /* Read the related interrupt status bits of this channel */

                uUsbHcintReg = (UINT32) pHCDData->uHCINTnStatus[uChannel];

                /* Clear this channel's interrupt status bits */

                vxAtomicAnd((atomic_t *)&pHCDData->uHCINTnStatus[uChannel],
                            (atomicVal_t)(~pHCDData->uHCINTnStatus[uChannel]));

                /* Clear the HAINT status's related bit */

                (void) vxAtomicAnd((atomic_t *)&pHCDData->uHAINTStatus,
                                   (atomicVal_t)(~(0x01 << uChannel)));

                uUsbHaintReg = (UINT32)pHCDData->uHAINTStatus;

                SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

                usbSynopsysHcdChannelInterruptHandler(pHCDData,
                                                      uChannel,
                                                      uUsbHcintReg);
                }
            }

        /* SOF interrupt */

        if (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_SOF)
            {

            SPIN_LOCK_ISR_TAKE(
                         &spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            vxAtomicAnd((atomic_t *)&pHCDData->uInterruptStatus,
                        (atomicVal_t)~USB_SYNOPSYSHCD_GINTSTS_SOF);

            SPIN_LOCK_ISR_GIVE(
                         &spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            /* Schedule if there is any request can be handled */

            usbSynopsysHcdProcessScheduleHandler(pHCDData);

            /* Mark SOF interrupt if no request should be server */

            if (0 == (lstCount(&(pHCDData->periodicReqList)) +
                      lstCount(&(pHCDData->periodicReqReadyList)) +
                      lstCount(&(pHCDData->nonPeriodicReqList)) +
                      lstCount(&(pHCDData->nonPeriodicReqReadyList))))
                {
                uUsbGinmsk = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                USB_SYNOPSYSHCD_GINTMSK);
                if (0 != (uUsbGinmsk & USB_SYNOPSYSHCD_GINTSTS_SOF))
                    {
                    uUsbGinmsk &= ~(USB_SYNOPSYSHCD_GINTSTS_SOF);
                    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                                USB_SYNOPSYSHCD_GINTMSK,
                                                uUsbGinmsk);
                    }
                }
            }
        }
    /* End of while () */
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdTransferHandler - handle messages sended by interrupt handler
*
* This thread routine handles the messages sended by interrup handler.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

VOID usbSynopsysHcdTransferHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    ssize_t  nBytes = 0x0;
    USB_SYNOPSYSHCD_TRANSFER_TASK_INFO transferTaskInfo;

    /* Check the validity of the parameter */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);

        return;
        }

    while (TRUE)
        {
        if (NULL == pHCDData->transferThreadMsgQ)
            break;

        /* Reset the transfer task information */

        OS_MEMSET(&transferTaskInfo, 0 ,
                  sizeof(USB_SYNOPSYSHCD_TRANSFER_TASK_INFO));

        /* Wait on the signalling of the event */

        nBytes = msgQReceive (pHCDData->transferThreadMsgQ,
                              (char*) & transferTaskInfo,
                              sizeof(USB_SYNOPSYSHCD_TRANSFER_TASK_INFO),
                              WAIT_FOREVER);

        if (nBytes != sizeof(USB_SYNOPSYSHCD_TRANSFER_TASK_INFO))
            continue;

        if (USB_SYNOPSYSHCD_TRANSFER_CMD_START ==
             transferTaskInfo.uCmdCode )
            {
            (void) OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID,
                                     WAIT_FOREVER);
            usbSynopsysHcdChannelStartProcess(transferTaskInfo.pHCDData,
                                        transferTaskInfo.pHCDPipe,
                                        (UINT8)transferTaskInfo.uDmaChannel);
            (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
            }
        else if (USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE ==
                  transferTaskInfo.uCmdCode)
            {
            (void) OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER);
            usbSynopsysHcdCompleteProcess(transferTaskInfo.pHCDData,
                                          transferTaskInfo.pHCDPipe,
                                          transferTaskInfo.pRequestInfo,
                                          transferTaskInfo.uCompleteStatus);
            (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
            }

        }
    /* End of while () */
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdCompleteProcess - handle the complete process to the request
*
* This routine does complete process of the request.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

VOID usbSynopsysHcdCompleteProcess
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,       /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,       /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,   /* Pointer to the request */
    USBHST_STATUS                 uCompleteStatus /* Complete Status */
    )
    {
    /* Pointer to ISO Packet Descriptor */

    pUSBHST_ISO_PACKET_DESC pIsoPacketDesc = NULL;

    /* Flag to check need schedule or not after complete this request */

    UINT8                   beScheduled = 0x0;
    pUSBHST_URB             pURB = NULL;

    pUSB_SYNOPSYSHCD_REQUEST_INFO pHeadRequest = NULL;

    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("parameters not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    if ((pURB = pRequestInfo->pUrb) != NULL)
        {
        /*
         * Isochronous transactions need extra processing as they might not be
         * done after a single data transfer
         */

        if (pHCDPipe->uEndpointType == USB_ATTR_ISOCH)
            {
            /* Get the first ISO packet descriptor */

            pIsoPacketDesc = (pUSBHST_ISO_PACKET_DESC )
                             pURB->pTransferSpecificData;

            /* Get the current ISO packet descriptor */

            pIsoPacketDesc =
                    &pIsoPacketDesc[pRequestInfo->uUrbTotalPacketCount - \
                                    pURB->uNumberOfPackets];

            if (pIsoPacketDesc != NULL)
                {
                /* Update the number of bytes transfered in this ISO packet */

                pIsoPacketDesc->uLength = pRequestInfo->uActLength;
                pIsoPacketDesc->nStatus =  uCompleteStatus;

                /*
                 * The judgement here must be > 1 not >= 1
                 * otherwise there will be some noise when play music
                 */

                if ((pURB->uNumberOfPackets > 1) &&
                    (uCompleteStatus == USBHST_SUCCESS))
                    {
                    /* Exclusively access the request list */

                    (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID,
                                            OS_WAIT_INFINITE);

                    pRequestInfo->uActLength = 0;
                    pRequestInfo->pUrb->uNumberOfPackets--;
                    pRequestInfo->uStage =
                        USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL;

                    /* Move the ready list back to send list and wait to schedule */

                    (void) usbSynopsysHcdMoveRequetToSendList(pHCDData,
                                                       pHCDPipe,
                                                       pRequestInfo);

                    /* Release the DMA channel resource */

                    pRequestInfo->uTransferFlag &=
                        ~USB_SYNOPSYSHCD_REQUEST_STILL_TRANSFERING;
                    pHCDPipe->uPipeFlag &= ~USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED;

                    if (pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] ==
                        pHCDPipe)
                        {
                        pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] = NULL;

                        (void) usbShdrcHsDmaChannelRelease(pHCDData,
                                   pHCDPipe->uDmaChannel);
                        }

                    /* Release the event */

                    (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

                    /*
                     * Now, there may be some requests which can be scheduled.
                     * We don't need to call the schedule function as the
                     * SOF interrupt will call it .
                     * usbSynopsysHcdProcessScheduleHandler(pHCDData);
                     */
                    return;
                    }
                }
            }
        }

    /* Exclusively access the request list */

    (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, OS_WAIT_INFINITE);

    /* Get the Head Request of the list */

    pHeadRequest = usbSynopsysHcdFirstReqGet(pHCDPipe);

    /* Determine if this transaction was scheduled when complete was called */

    beScheduled = (pHCDPipe->uPipeFlag & USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED) &&
                  (pHeadRequest == pRequestInfo);

    /* Delete request From related list */

    usbSynopysHcdDeleteRequestFromReadyList(pHCDData,
                                            pHCDPipe,
                                            pRequestInfo);

    /* Release the DMA channel resource mark the transfer stop transfer */

    pRequestInfo->uTransferFlag &= ~USB_SYNOPSYSHCD_REQUEST_STILL_TRANSFERING;
    pHCDPipe->uPipeFlag &= ~USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED;

    if (pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] == pHCDPipe)
        {
        pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] = NULL;

        (void) usbShdrcHsDmaChannelRelease(pHCDData, pHCDPipe->uDmaChannel);
        }

    /* Update URB's status */

    if ((pURB != NULL) && (pURB->pHcdSpecific != NULL))
        {
        pURB->uTransferLength =
            min(pRequestInfo->uActLength, pURB->uTransferLength);

        if (pHCDPipe->uEndpointDir != USB_SYNOPSYSHCD_DIR_OUT)
            CACHE_USER_INVALIDATE(pURB->pTransferBuffer, pURB->uTransferLength);

        usbSynopsysHcdUrbComplete(pURB, uCompleteStatus);
        pRequestInfo->pUrb = NULL;
        }

    /* Remove the transaction from the pipe list */

    usbSynopsysHcdDeleteReq(pHCDPipe, pRequestInfo);

    /* Check is there some requests can be scheduled */

    if (beScheduled)
        {
        usbSynopsysHcdProcessScheduleHandler(pHCDData);
        }

    /* Release the exclusive access */

    (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

    }

/*******************************************************************************
*
* usbSynopsysHcdCopyRHInterruptData - copy the interrupt status data
*
* This function is used to copy the interrupt status data
* to the request buffer if a request is pending or to
* the interrupt status buffer if a request is not pending.
*
* RETURNS: TRUE, or FALSE if the data is not successfully copied.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

BOOLEAN usbSynopsysHcdCopyRHInterruptData
    (
    pUSB_SYNOPSYSHCD_DATA  pHCDData,     /* Pointer to the pHCDData structure */
    UINT32                 uStatusChange /* Status change data */
    )
    {
    /* To hold the pointer to the request information */

    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo = NULL;

    /* Interrupt data */

    UINT32 uInterruptData = 0;

    /* Index of the host controller */

    UINT32    uBusIndex = 0;

    /* Check the validity of the parameters */

    if ((NULL == pHCDData) || (0 == uStatusChange))
        {
        USB_SHCD_ERR("parameters not valid\n", 0, 0, 0, 0, 0, 0);

        return FALSE;
        }

    /* Extract the index of the host controller */

    uBusIndex = pHCDData->uBusIndex;

    /*
     * If there is any request which is pending for the root hub
     * interrupt endpoint, populate the URB.
     * This has to be done by exclusively accessing the request list.
     */

    (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER);

    /* Copy the status information which is stored already */

    OS_MEMCPY(&uInterruptData,
              pHCDData->RHData.pHubInterruptData,
              pHCDData->RHData.uSizeInterruptData);

    /* Swap the data to CPU endian format */

    uInterruptData = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                   uInterruptData);

    uInterruptData |= uStatusChange;

    /* Swap it back to LE */

    uInterruptData = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uInterruptData);

    /* Copy the data to the interrupt data buffer */

    OS_MEMCPY(pHCDData->RHData.pHubInterruptData,
              &uInterruptData,
              pHCDData->RHData.uSizeInterruptData);

    /*
     * This condition will pass when devices are kept connected
     * when the system is booted
     */

    if (NULL == pHCDData->RHData.pInterruptPipe)
        {
        /* Release the exclusive access */

        (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
        return TRUE;
        }

    /* Get the Head request of the list */

    pRequestInfo = usbSynopsysHcdFirstReqGet(pHCDData->RHData.pInterruptPipe);

    /*
     * If there is no request pending, copy the data
     * to the interrupt data buffer
     */

    if (NULL != pRequestInfo)
        {
        USB_SHCD_WARN("usbSynopsysHcdCopyRHInterruptData - "
                      "Calling callback :uStatusChange = 0x%x\n",
                      uStatusChange, 0, 0, 0, 0, 0);

        if (NULL == pRequestInfo->pUrb ||
            NULL == pRequestInfo->pUrb->pTransferBuffer)
            {
            /* Release the exclusive access */

            (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
            return FALSE;
            }

        /* Swap the data to LE */

        uStatusChange = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uStatusChange);

        /* Populate the data buffer */

        OS_MEMCPY(pRequestInfo->pUrb->pTransferBuffer,
                  &uStatusChange,
                  pHCDData->RHData.uSizeInterruptData);

        /* Update the URB's transfer length */

        pRequestInfo->pUrb->uTransferLength =
                        pHCDData->RHData.uSizeInterruptData;

        usbSynopsysHcdUrbComplete(pRequestInfo->pUrb, USBHST_SUCCESS);

        usbSynopsysHcdDeleteReq(pHCDData->RHData.pInterruptPipe, pRequestInfo);
        }

    /* Release the exclusive access */

    (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

    return TRUE;
    }

/*******************************************************************************
*
* usbSynopsysHcdProcessScheduleHandler - schedule the transfer of the hcd
*
* This routine schedules the transfer of the hcd, parapers the related
* information before transfer the requeset.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

VOID usbSynopsysHcdProcessScheduleHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    UINT8  uChannel = 32;
    pUSB_SYNOPSYSHCD_PIPE   pHCDPipe = NULL;
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo = NULL;
    NODE *  pNode = NULL;

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /* If there isn't any request in send list need schedule just return */

    if (0 == (lstCount(&(pHCDData->periodicReqList)) +
             lstCount(&(pHCDData->nonPeriodicReqList))))
        {
        USB_SHCD_VDBG("usbSynopsysHcdProcessScheduleHandler - "
                     "No request in sending list \n", 0, 0, 0, 0, 0, 0);
        return;
        }

    (void) OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER);

    /*
     * Periodic request have higher PRI than the Non-periodic request
     * So we start to schedule the periodic request first
     */

    pNode = lstFirst(&(pHCDData->periodicReqList));
    while (pNode != NULL)
        {
        pRequestInfo = SCHED_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        if (NULL != pRequestInfo)
            {
            pHCDPipe = usbSynopsysHcdCheckSchedule(pHCDData,
                                                   pRequestInfo->pHCDPipe);
            if (NULL != pHCDPipe)
                {
                /*
                 * This entry will mark the related channel to busy,
                 * and start deal with the request
                 */

                /* Get an idle channel to service */

                uChannel = (UINT8)usbShdrcHsDmaFreeChannelRequest(pHCDData);

                if (uChannel == USB_SHDRC_DMA_INVALID_CHANNEL)
                    {
                    USB_SHCD_ERR("usbSynopsysHcdProcessScheduleHandler - "
                                 "No idle DMA channel. \n", 0, 0, 0, 0, 0, 0);
                    (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
                    return;
                    }

                usbSynopsysHcdChannelStartProcess(pHCDData,
                                                  pHCDPipe,
                                                  uChannel);
                pHCDPipe = NULL;
                }
            }
        pNode = lstNext(pNode);
        }

    /*
     * After schedule the periodic request,
     * check non-periodic request need to be serviced or not
     */

    pNode = lstFirst(&(pHCDData->nonPeriodicReqList));
    while (pNode != NULL)
        {
        pRequestInfo = SCHED_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        if (NULL != pRequestInfo)
            {
            pHCDPipe = usbSynopsysHcdCheckSchedule(pHCDData,
                                                   pRequestInfo->pHCDPipe);
            if (NULL != pHCDPipe)
                {
                /*
                 * This entry will mark the related channel to busy,
                 * and start deal with the request
                 */

                /* Get an idle channel to service */

                uChannel = (UINT8)usbShdrcHsDmaFreeChannelRequest(pHCDData);

                if (uChannel == USB_SHDRC_DMA_INVALID_CHANNEL)
                    {
                    USB_SHCD_ERR("usbSynopsysHcdProcessScheduleHandler - "
                                 "No idle DMA channel. \n", 0, 0, 0, 0, 0, 0);
                    (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
                    return;
                    }

                usbSynopsysHcdChannelStartProcess(pHCDData,
                                                  pHCDPipe,
                                                  uChannel);
                pHCDPipe = NULL;
                }
            }
        pNode = lstNext(pNode);
        }
    (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdFlushRequest - complete all requests with USBHST_FAILURE
*
* This function should be called when device disconnect from root hub. It will
* complete all requests with USBHST_FAILURE.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

VOID usbSynopsysHcdFlushRequest
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* Pointer to HCD block */
    )
    {
    NODE *pNode = NULL;
    pUSB_SYNOPSYSHCD_PIPE             pHCDPipe = NULL;
    pUSB_SYNOPSYSHCD_REQUEST_INFO     pRequestInfo = NULL;

    /*
     * This driver is driven by SOF interrupt. If device is pulged out from
     * root hub, there will be no SOF interrupt, so all the requests in the
     * schedule list should be flushed out and be completed
     * in "USBHST_FAILURE" status.
     */

    /* Move all request from sending queue to ready queue */

    (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER);

    pNode = lstFirst(&(pHCDData->periodicReqList));
    while (pNode != NULL)
        {
        pRequestInfo = SCHED_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        pHCDPipe = pRequestInfo->pHCDPipe;
        (void) usbSynopsysHcdMoveRequetToReadyList(pHCDData,
                                                   pHCDPipe,
                                                   pRequestInfo);
        pNode = lstNext(pNode);
        }

    pNode = lstFirst(&(pHCDData->nonPeriodicReqList));
    while (pNode != NULL)
        {
        pRequestInfo = SCHED_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        pHCDPipe = pRequestInfo->pHCDPipe;
        (void) usbSynopsysHcdMoveRequetToReadyList(pHCDData,
                                                   pHCDPipe,
                                                   pRequestInfo);
        pNode = lstNext(pNode);
        }

    /* Complete all the ready request with USBHST_FAILURE */

    pNode = lstFirst(&(pHCDData->periodicReqReadyList));
    while (pNode != NULL)
        {
        pRequestInfo = SCHED_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        if (pRequestInfo != NULL)
            {
            pHCDPipe = pRequestInfo->pHCDPipe;
            usbSynopsysHcdCompleteProcess(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          USBHST_FAILURE);
            }
        pNode = lstNext(pNode);
        }

    pNode = lstFirst(&(pHCDData->nonPeriodicReqReadyList));
    while (pNode != NULL)
        {
        pRequestInfo = SCHED_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        if (pRequestInfo != NULL)
            {
            pHCDPipe = pRequestInfo->pHCDPipe;
            usbSynopsysHcdCompleteProcess(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          USBHST_FAILURE);
            }

        pNode = lstNext(pNode);
        }

    (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdFillTaskInfoAndSend - fill and send a message
*
* This routine fills a message and send it to the MsgQ handled by the
* transfer thread.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL __inline__ VOID usbSynopsysHcdFillTaskInfoAndSend
    (
    pUSB_SYNOPSYSHCD_DATA          pHCDData,        /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE          pHCDPipe,        /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO  pRequestInfo,    /* Pointer to the request */
    UINT32                         uChannel,        /* Transfer DMA channel */
    UINT32                         uCmdCode,        /* Command code */
    USBHST_STATUS                  uCompleteStatus  /* Complete Status */
    )
    {
    USB_SYNOPSYSHCD_TRANSFER_TASK_INFO transferTaskInfo;

    if ((NULL == pHCDData) ||
        (NULL == pHCDData->transferThreadMsgQ ))
        {
        USB_SHCD_ERR("MsgQueue doesn't exist\n", 0, 0, 0, 0, 0, 0);
        return ;
        }

    transferTaskInfo.pHCDData = pHCDData;
    transferTaskInfo.pHCDPipe = pHCDPipe;
    transferTaskInfo.pRequestInfo = pRequestInfo;
    transferTaskInfo.uDmaChannel = uChannel;
    transferTaskInfo.uCompleteStatus = uCompleteStatus;
    transferTaskInfo.uCmdCode = uCmdCode;

    if (OK !=  msgQSend (pHCDData->transferThreadMsgQ,
                         (char*) &transferTaskInfo,
                         sizeof(USB_SYNOPSYSHCD_TRANSFER_TASK_INFO),
                         WAIT_FOREVER,
                         MSG_PRI_NORMAL))
        {
        USB_SHCD_ERR("Message Send Failed\n", 0, 0, 0, 0, 0, 0);
        return;
        }
    return ;
    }

/*******************************************************************************
*
* usbSynopsysHcdPortChangeHandler - handle the port change interrupt
*
* This routine handles a port change interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdPortChangeHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* Pointer to HCD block */
    )
    {
    /* To hold the HPRT status of the hub */

    UINT32 uHprtStatus = 0x0;

    /* To hold the status change of the hub */

    UINT32 uStatusChange = 0;

    /* To hold the Port Status Change information */

    UINT32 uPortStatus = 0;


    /* Check the validity of the parameter */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);

        return;
        }

    /* Check the validity of the request synchronization event */

    if (NULL == pHCDData->RequestSynchEventID)
        {
        USB_SHCD_ERR("RequestSynchEventID not valid\n", 0, 0, 0, 0, 0, 0);

        return;
        }

    SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

    /* Currently, there is only one port */

    uHprtStatus = (UINT32)pHCDData->uHPRTStatus;

    /* Copy the data in the Root Hub's buffer */

    OS_MEMCPY(&uPortStatus,
              pHCDData->RHData.pPortStatus,
              USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

    SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

    /* Swap the data to the LE format */

    uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(pHCDData->uBusIndex,
                                                uPortStatus);

    /* Translate the status of PHRT to the hub port status */

    /* The first 2 bytes contain the port status */

    /* Port connected */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTCONNSTS)
        {
        uPortStatus |=  USB_HUB_STS_PORT_CONNECTION;
        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        (void) vxAtomicAnd((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)~USB_SYNOPSYSHCD_HPRT_PRTCONNSTS);
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        }
    else
        {
        pHCDData->uMicroFrameNumChangeFlag = 0;
        }

    /* Port enable */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTENA)
        {
        uPortStatus |=  USB_HUB_STS_PORT_ENABLE;
        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        (void) vxAtomicAnd((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)~USB_SYNOPSYSHCD_HPRT_PRTENA);
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        }

    /* Port suspend or resume */

    if ((uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTSUSP) ||
        (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTRES))
        {
        uPortStatus |=  USB_HUB_STS_PORT_SUSPEND;
        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        (void) vxAtomicAnd((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)(~(USB_SYNOPSYSHCD_HPRT_PRTRES|
                                           USB_SYNOPSYSHCD_HPRT_PRTSUSP)));
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        }

    /* Port over current */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTOVRCURRACT)
        {
        uPortStatus |=  USB_HUB_STS_PORT_OVER_CURRENT;
        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        (void) vxAtomicAnd((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)~USB_SYNOPSYSHCD_HPRT_PRTOVRCURRACT);
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        }

    /* Port reset */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTRST)
        {
        uPortStatus |=  USB_HUB_STS_PORT_RESET;
        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        (void) vxAtomicAnd((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)~USB_SYNOPSYSHCD_HPRT_PRTRST);
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        }

    /* Port power */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTPWR)
        {
        uPortStatus |=  USB_HUB_STS_PORT_POWER;
        SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        (void) vxAtomicAnd((atomic_t *)&pHCDData->uHPRTStatus,
                           (atomicVal_t)~USB_SYNOPSYSHCD_HPRT_PRTPWR);
        SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
        }

    /* Port speed */

    if ((uHprtStatus & (USB_SYNOPSYSHCD_HPRT_PRTSPD ))
              == USB_SYNOPSYSHCD_HPRT_PRTSPD_HIGH)
        {
        uPortStatus |= USB_HUB_STS_PORT_HIGH_SPEED;
        uPortStatus &= (UINT32) ~USB_HUB_STS_PORT_LOW_SPEED;
        }
    if ((uHprtStatus & (USB_SYNOPSYSHCD_HPRT_PRTSPD ))
              == USB_SYNOPSYSHCD_HPRT_PRTSPD_FULL)
        {
        uPortStatus &= (UINT32) ~USB_HUB_STS_PORT_HIGH_SPEED;
        uPortStatus &= (UINT32) ~USB_HUB_STS_PORT_LOW_SPEED;
        }
    if ((uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTSPD )
              == USB_SYNOPSYSHCD_HPRT_PRTSPD_LOW)
        {
        uPortStatus &= (UINT32) ~USB_HUB_STS_PORT_HIGH_SPEED;
        uPortStatus |= USB_HUB_STS_PORT_LOW_SPEED;
        }

    /*
     * The following bits of pHCDData->uHPRTStatus no need to be cleard.
     * Clear feature command will clear these bits later, until these bits
     * be cleared we should always return the change bits.
     */

    /* Port change status - connection change */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTCONNDET)
        {
        uPortStatus |= (USB_HUB_C_PORT_CONNECTION << 16);
        uStatusChange |= USB_SYNOPSYSHCD_RH_PORT_CHANGE_MASK;
        }

    /*
     * Disconnect interrupt - we should use enable changes status to
     * let this devive be deleted.
     */

    if (pHCDData->uInterruptStatus & (atomic_t)USB_SYNOPSYSHCD_GINTSTS_DISCONNINT)
        {
        pHCDData->uMicroFrameNumChangeFlag = 0;
        uPortStatus |= (USB_HUB_C_PORT_ENABLE << 16);
        uStatusChange |= USB_SYNOPSYSHCD_RH_PORT_CHANGE_MASK;
        usbSynopsysHcdFlushRequest(pHCDData);
        }

    /* Port change status - enable change */

    /*
     * The enable change means the port from enable to disable
     * So, if the change happens we can report the status
     * when port is disable or else the hub will remove the device
     */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTENCHNG)
        {
        if ((uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTENA)== 0x0)
            {
            pHCDData->uMicroFrameNumChangeFlag = 0;
            uPortStatus |= (USB_HUB_C_PORT_ENABLE << 16);
            uStatusChange |= USB_SYNOPSYSHCD_RH_PORT_CHANGE_MASK;
            usbSynopsysHcdFlushRequest(pHCDData);
            }
        }

    /* Port change status - overcurrent change */

    if (uHprtStatus & USB_SYNOPSYSHCD_HPRT_PRTOVRCURRCHNG)
        {
        uPortStatus |= (USB_HUB_C_PORT_OVER_CURRENT << 16);
        uStatusChange |= USB_SYNOPSYSHCD_RH_PORT_CHANGE_MASK;
        }

    /* Swap the data to LE format */

    uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(pHCDData->uBusIndex,
                                                uPortStatus);

    /* Update the pPortStatus */

    OS_MEMCPY(pHCDData->RHData.pPortStatus,
              &uPortStatus,
              USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

    /* If there is no change return */

    if ((0 == uStatusChange) ||
        (NULL == pHCDData->RHData.pHubInterruptData))
        {
        USB_SHCD_WARN("usbSynopsysHcdPortChangeHandler - "
                      "No change in the hub status \n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /*
     * Call the function to copy the interrupt status to the
     * request buffer if a request is pending or to the Root hub interrupt
     * transfer data buffer if a request is not pending
     */

    /* Nothing can be done when the return value is false */

    (void) usbSynopsysHcdCopyRHInterruptData(pHCDData,
                                      uStatusChange);

    return;
    }

#if defined(_WRS_CONFIG_ALTERA_SOC_STRATIX10_VP)
/*******************************************************************************
*
* usbSynopsysDwcHcdChannelProcessXFERCOMPLResponse - handle the channel's XFERCOMPL interrupt
*
* This routine handles the Channel's XFERCOMPL interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysDwcHcdChannelProcessXFERCOMPLResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,              /* Ptr to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,              /* Ptr to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,          /* Ptr to the request */
    UINT8                         uChannel,              /* DMA channel
                                                          * for transfer
                                                          */
    UINT32                        uLengthLeftToTransfer, /* Left length
                                                          * to transfer
                                                          */
    UINT32                        uLengthThisTransfer    /* Length in this
                                                          * transfer
                                                          */
    )
    {
    UINT32 uUsbHprtReg;

    if ((NULL == pHCDData) || (NULL == pHCDPipe) ||
        (NULL == pRequestInfo) || (NULL == pRequestInfo->pUrb))
        {
        return;
        }

    /*
    Below is workaround to make possible work on virtual SoC emulator.
    We have to handle there XFERCOMPL bit because ACK bit is not set by the emulator.
    */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_INTERRUPT:
        case USB_ATTR_ISOCH:
            if (uLengthLeftToTransfer == 0)
                {
                usbSynopsysHcdChannelProcessACKResponse(pHCDData,
                                                        pHCDPipe,
                                                        pRequestInfo,
                                                        uChannel,
                                                        uLengthLeftToTransfer,
                                                        uLengthThisTransfer);
                }
            else
                {
                usbSynopsysHcdChannelProcessNAKResponse(pHCDData,
                                                        pHCDPipe,
                                                        pRequestInfo);
                }
            break;

        case USB_ATTR_CONTROL:
        case USB_ATTR_BULK:
        default:
             uUsbHprtReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                      USB_SYNOPSYSHCD_HPRT);

             /* Judge whether the device is connect or not */

            if (uUsbHprtReg & USB_SYNOPSYSHCD_HPRT_PRTCONNSTS)
                {
                usbSynopsysHcdChannelProcessACKResponse(pHCDData,
                                                        pHCDPipe,
                                                        pRequestInfo,
                                                        uChannel,
                                                        uLengthLeftToTransfer,
                                                        uLengthThisTransfer);
                }
            else
                {
                usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                  pHCDPipe,
                                                  pRequestInfo,
                                                  uChannel,
                                                  USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                  USBHST_FAILURE);
                }
        }
    }
#endif /* _WRS_CONFIG_ALTERA_SOC_STRATIX10_VP */

/*******************************************************************************
* usbSynopsysHcdChannelProcessACKResponse - handle the channel's ACK interrupt
*
* This routine handles the Channel's ACK interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelProcessACKResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,              /* Ptr to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,              /* Ptr to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,          /* Ptr to the request */
    UINT8                         uChannel,              /* DMA channel
                                                          * for transfer
                                                          */
    UINT32                        uLengthLeftToTransfer, /* Left length
                                                          * to transfer
                                                          */
    UINT32                        uLengthThisTransfer    /* Length in this
                                                          * transfer
                                                          */
    )
    {
    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET          pSetup       = NULL;

    /* Parameter varification */

    if ((NULL == pHCDData) || (NULL == pHCDPipe) ||
        (NULL == pRequestInfo) || (NULL == pRequestInfo->pUrb))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    pRequestInfo->uRetried = 0;

    /* Since we got an ACK, the pipe don't need to do a ping, unmask it */

    pHCDPipe->uPipeFlag &= ~USB_SYNOPSYSHCD_PIPE_FLAG_NEED_PING;

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_CONTROL:
            switch (pRequestInfo->uStage)
                {
                case USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL:
                case USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE:

                    /* This should be impossible */

                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      pRequestInfo,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                      USBHST_FAILURE);
                    break;

                case USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP:

                    pHCDPipe->uPidToggle = 1;

                    if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
                        {
                        pRequestInfo->uStage =
                           USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP_SPLIT_COMPLETE;
                        }
                    else
                        {
                        pSetup = (pUSBHST_SETUP_PACKET)
                              pRequestInfo->pUrb->pTransferSpecificData;
                        /*
                         * Here we do not change to LE, since we do not want to
                         * know the existly length
                         */

                        if (pSetup->wLength)
                            {
                            pRequestInfo->uStage =
                                USB_SYNOPSYSHCD_REQUEST_STAGE_DATA;
                            }
                        else
                            {
                            pRequestInfo->uStage =
                                USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS;
                            }
                        }

                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          NULL,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                          0);
                    break;
                case USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP_SPLIT_COMPLETE:

                    pSetup = (pUSBHST_SETUP_PACKET)
                             pRequestInfo->pUrb->pTransferSpecificData;
                    /*
                     * Here we do not change to LE, since we do not want to
                     * know the existly length
                     */

                    if (pSetup->wLength)
                        {
                        pRequestInfo->uStage =
                            USB_SYNOPSYSHCD_REQUEST_STAGE_DATA;
                        }
                    else
                        {
                        pRequestInfo->uStage =
                            USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS;
                        }

                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      NULL,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                      0);
                    break;
                case USB_SYNOPSYSHCD_REQUEST_STAGE_DATA:
                    if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
                        {
                        pRequestInfo->uStage  =
                            USB_SYNOPSYSHCD_REQUEST_STAGE_DATA_SPLIT_COMPLETE;
                        }
                    else if (uLengthLeftToTransfer == 0)
                        {
                        pHCDPipe->uPidToggle = 1;
                        pRequestInfo->uStage = USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS;
                        }
                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      NULL,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                      0);
                    break;

                case USB_SYNOPSYSHCD_REQUEST_STAGE_DATA_SPLIT_COMPLETE:
                    if (uLengthLeftToTransfer == 0)
                        {
                        pHCDPipe->uPidToggle = 1;
                        pRequestInfo->uStage = USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS;
                        }
                    else
                        {
                        pRequestInfo->uStage = USB_SYNOPSYSHCD_REQUEST_STAGE_DATA;
                        }

                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      NULL,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                      0);
                    break;

                case USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS:
                    if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
                        {
                        pRequestInfo->uStage =
                            USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS_SPLIT_COMPLETE;
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          NULL,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                          0);
                        }
                    else
                        {
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          pRequestInfo,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                          USBHST_SUCCESS);
                        }
                    break;

                case USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS_SPLIT_COMPLETE:
                     usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                       pHCDPipe,
                                                       pRequestInfo,
                                                       uChannel,
                                                       USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                       USBHST_SUCCESS);
                    break;
                }
            break;
        case USB_ATTR_BULK:
        case USB_ATTR_INTERRUPT:
            if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
                {
                if (pRequestInfo->uStage ==
                    USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE)
                    {
                    if (uLengthLeftToTransfer &&
                        (uLengthThisTransfer >= pHCDPipe->uMaximumPacketSize))
                        {
                        pRequestInfo->uStage = USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL;
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          NULL,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                          0);
                        }
                    else
                        {
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          pRequestInfo,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                          USBHST_SUCCESS);
                        }
                    }
                else
                    {
                    pRequestInfo->uStage =
                        USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE;
                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      NULL,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                      0);
                    }
                }
            else
                {
                /*
                 * When we do out transfer with full speed hub,
                 * sometimes we will get ack interrupt even the transfer
                 * is not complete. So we should judge it and
                 * resend the remaining data.
                 */

                if ((uLengthLeftToTransfer >0 )&&
                     (uLengthThisTransfer >= pHCDPipe->uMaximumPacketSize))
                    {
                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      NULL,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                      0);
                    }
                else
                    {
                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      pRequestInfo,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                      USBHST_SUCCESS);

                    }
                }
            break;
        case USB_ATTR_ISOCH:
            if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
                {
                /*
                 * ISOCHRONOUS OUT splits don't require a complete split stage.
                 * Instead they use a sequence of begin OUT splits to transfer
                 * the data 188 bytes at a time. Once the transfer is complete,
                 * the pipe sleeps until the next schedule interval
                 */

                if (pHCDPipe->uEndpointDir == USB_SYNOPSYSHCD_DIR_OUT)
                    {
                    /*
                     * If no space left or this wasn't a max size packet then
                     * this transfer is complete. Otherwise start it again
                     * to send the next 188 bytes
                     */

                    if (!uLengthLeftToTransfer || (uLengthThisTransfer < 188))
                        {
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          pRequestInfo,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                          USBHST_SUCCESS);
                        }
                    else
                        {
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          NULL,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                          0);
                        }
                    }
                else
                    {
                    if (pRequestInfo->uStage ==
                            USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE)
                        {
                        /*
                         * Complete split stage in the incomming data phase.
                         * keep getting data until we run out of space or
                         * get a small packet
                         */

                        if (uLengthLeftToTransfer &&
                            (uLengthThisTransfer >= pHCDPipe->uMaximumPacketSize))
                            {
                            usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                              pHCDPipe,
                                                              NULL,
                                                              uChannel,
                                                              USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                              0);
                            }
                        else
                            {
                            usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                              pHCDPipe,
                                                              pRequestInfo,
                                                              uChannel,
                                                              USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                              USBHST_SUCCESS);
                            }
                        }
                    else
                        {
                        pRequestInfo->uStage =
                            USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE;
                        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                          pHCDPipe,
                                                          NULL,
                                                          uChannel,
                                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                          0);
                    }
                    }
                }
            else
                {
                usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                  pHCDPipe,
                                                  pRequestInfo,
                                                  uChannel,
                                                  USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                  USBHST_SUCCESS);

                }

            break;
        }
     return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelProcessNAKResponse - handle the channel's NAK interrupt
*
* This rountine handles the channel's NAK interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelProcessNAKResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,     /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo  /* Pointer to the request */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }
    /*
     * NAK as a response means the device couldn't accept the transaction,
     * but it should be retried in the future. Just resubmit it, but do a
     * full schedule. This way periodic requests wait for their next
     * interval and other pipes might get a chance
     */

    if (pHCDPipe->uEndpointType != USB_ATTR_ISOCH)
        {
        (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, OS_WAIT_INFINITE);

        /*
         * Rewind to the beginning of the transaction by anding off the
         * split complete bit
         */

        pRequestInfo->uStage =
                pRequestInfo->uStage & USB_SYNOPSYSHCD_REQUEST_STAGE_INIT_MASK;

        /*
         * Move request from reday list to send list
         * wait SOF interrrupt to schedule the request
         */

        (void) usbSynopsysHcdMoveRequetToSendList(pHCDData,
                                           pHCDPipe,
                                           pRequestInfo);

        /* Release the DMA channel resource */

        pRequestInfo->uTransferFlag &= ~USB_SYNOPSYSHCD_REQUEST_STILL_TRANSFERING;
        pHCDPipe->uPipeFlag &= ~USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED;

        if  (pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] == pHCDPipe)
            {
            pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] = NULL;

            (void) usbShdrcHsDmaChannelRelease(pHCDData, pHCDPipe->uDmaChannel);
            }
        (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
        }
    else
        {
        USB_SHCD_DBG("usbSynopsysHcdChannelProcessNAKResponse - "
                     "ISO transfer should never into nak interrupt\n",
                     0, 0, 0, 0, 0, 0);
        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelProcessNYETResponse - handle the channel's NYET interrupt
*
* This routine handles the channel's NYET interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelProcessNYETResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,              /* Ptr to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,              /* Ptr to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,          /* Ptr to the request */
    UINT8                         uChannel,              /* DMA channel
                                                          * for transfer
                                                          */
    UINT32                        uLengthLeftToTransfer, /* Left length
                                                          * to transfer
                                                          */
    UINT32                        uLengthThisTransfer    /* Length in this
                                                          * transfer
                                                          */
    )
    {
    UINT16 uMicroFrameNum = 0x0;
    UINT16 uFullFrameNum = 0x0;

    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    pRequestInfo->uRetried = 0;

    if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
        {
        /*
         * NYET on periodic splits need to be retried at the next interval.
         * Other transactions must be retried immediately
         */

        if (pHCDPipe->uEndpointType == USB_ATTR_INTERRUPT)
            {
            /* Get current micro frame number */

            (void) usbSynopsysHcdGetFrameNumber((UINT8)pHCDData->uBusIndex,
                                                &uMicroFrameNum);

            /* Get full frame number */

            uFullFrameNum = USB_SYNOPSYSHCD_GET_FULL_FRAME_NUMBER(uMicroFrameNum);

            /*
             * If not in the scheduled full frame we should reset the stage
             * and restart the schedule in next interval
             */

            if (uFullFrameNum != pHCDPipe->uFullFrameNumberLast)
                {
                (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID,
                                        OS_WAIT_INFINITE);
                (void) usbSynopsysHcdMoveRequetToSendList(pHCDData,
                                                   pHCDPipe,
                                                   pRequestInfo);

                pRequestInfo->uTransferFlag &=
                    ~USB_SYNOPSYSHCD_REQUEST_STILL_TRANSFERING;
                pHCDPipe->uPipeFlag &= ~USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED;

                if (pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] == pHCDPipe)
                    {
                    pHCDData->pPipeInChannel[pHCDPipe->uDmaChannel] = NULL;

                    (void) usbShdrcHsDmaChannelRelease(pHCDData, pHCDPipe->uDmaChannel);
                    }
                pRequestInfo->uStage &= USB_SYNOPSYSHCD_REQUEST_STAGE_INIT_MASK;
                (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
                }
            else
                {
                usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                  pHCDPipe,
                                                  NULL,
                                                  uChannel,
                                                  USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                                  0);

                }
            }
        else
            {
            usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                              pHCDPipe,
                                              NULL,
                                              uChannel,
                                              USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                              0);
            }
        }
    else
        {
        /*
         * If there is more data to go then we need to try again.
         * Otherwise this transaction is complete
         */

        if ((uLengthLeftToTransfer > 0) &&
            (uLengthThisTransfer >= pHCDPipe->uMaximumPacketSize))
            {
            usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                              pHCDPipe,
                                              NULL,
                                              uChannel,
                                              USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                              0);
            }
        else
            {
            usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                              pHCDPipe,
                                              pRequestInfo,
                                              uChannel,
                                              USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                              USBHST_SUCCESS);

            }

        }
        return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelProcessXACTERRResponse - handle the channel's XACTERR
* interrupt
*
* This routine handles the channel's XACTERR interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelProcessXACTERRResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,     /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo, /* Pointer to the request */
    UINT8                         uChannel      /* DMA transfer channel */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    pRequestInfo->uRetried ++;

    if (pRequestInfo->uRetried > 3)
        {
        /*
         * XactErr as a response means the device signaled something wrong
         * with the transfer.
         */

        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                          USBHST_DEVICE_NOT_RESPONDING_ERROR);

        }
    else
        {
        /*
         * Rewind to the beginning of the transaction by anding off the
         * split complete bit
         */

        pRequestInfo->uStage &= USB_SYNOPSYSHCD_REQUEST_STAGE_INIT_MASK;
        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          NULL,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                          USBHST_SUCCESS);

        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelProcessDATAGLERResponse - handle the channel's DATAGLERR
* interrupt
*
* This routine handles the channel's DATAGLERR interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelProcessDATAGLERResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,     /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo, /* Pointer to the request */
    UINT8                         uChannel      /* DMA transfer channel */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    pRequestInfo->uRetried ++;

    /*
     * If retry times > 3 should complet the URB with fail status,
     * or else just restart the transfer
     */

    if (pRequestInfo->uRetried > 3)
        {
        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                          USBHST_FAILURE);
        }
    else
        {
        /*
         * Rewind to the beginning of the transaction by anding off the
         * split complete bit
         */

        pRequestInfo->uStage &= USB_SYNOPSYSHCD_REQUEST_STAGE_INIT_MASK;
        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          NULL,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                          USBHST_SUCCESS);
        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelProcessFRMOVRUNResponse - handle the channel's FRMOVRUN
* interrupt
*
* This routine handles the channel's FRMOVRUN interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelProcessFRMOVRUNResponse
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,     /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo, /* Pointer to the request */
    UINT8                         uChannel      /* DMA transfer channel */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /*
     * For periodic transfer we need to restart the transfer in FRMOVRUN
     * interrupt. For non-periodic transfer we need to do nothing in
     * FRMOVRUN interrupt.
     */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_INTERRUPT:
        case USB_ATTR_ISOCH:
            pRequestInfo->uStage &= USB_SYNOPSYSHCD_REQUEST_STAGE_INIT_MASK;
            usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                              pHCDPipe,
                                              NULL,
                                              uChannel,
                                              USB_SYNOPSYSHCD_TRANSFER_CMD_START,
                                              USBHST_SUCCESS);
            break;
        case USB_ATTR_CONTROL:
        case USB_ATTR_BULK:
        default:
            break;
        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelInterruptHandler - handle all the channel's interrupt
*
* This routine handles all the channel's interrupt.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelInterruptHandler
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,    /* Pointer to HCD block */
    UINT8                 uChannel,    /* DMA transfer channel */
    UINT32                uUsbHcintReg /* HCINT register value */
    )
    {
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe     = NULL;
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo = NULL;
    pUSBHST_ISO_PACKET_DESC       pIsoPacketDesc = NULL;
    UINT32 uUsbHprtReg = 0x0;
    UINT32 uUsbHccharReg = 0x0;
    UINT32 uUsbHctsizReg = 0x0;
    UINT32 uLengthThisTransfer = 0;
    UINT32 uLengthLeftToTransfer = 0;
    UINT32 uEndPacket = 0;

    if (pHCDData == NULL)
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /* Make sure this channel is tied to a valid pipe */

    if (!(pHCDPipe = pHCDData->pPipeInChannel[uChannel]))
        {
        USB_SHCD_ERR("Pipe not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    (void) OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER);

    /* Un-halt the channel for this transfer */

    if (pHCDData->uPlatformType != USB_SYNOPSYSHCI_PLATFORM_ALTERA_SOC_GEN5
        && pHCDData->uPlatformType != USB_SYNOPSYSHCI_PLATFORM_RPI_3)
        {
        usbSynopsysHcdUnHaltChannel(pHCDData, pHCDPipe);
        }

    /* Get the first request of the pipe */

    pRequestInfo = usbSynopsysHcdFirstReqGet(pHCDPipe);

    if (pRequestInfo == NULL)
        {
        USB_SHCD_ERR("Request not valid,may be canceld\n", 0, 0, 0, 0, 0, 0);

        (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
        return;
        }

    /* Make sure the channel got the halt interrupt */

    if ((uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_CHHLTD) == 0)
        {
        USB_SHCD_DBG("usbSynopsysHcdChannelInterruptHandler - "
                     "We donn't get HALT interrupy in this channel\n",
                     0, 0, 0, 0, 0, 0);

        (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
        return;
        }

    /* Get the HCCHAR and HCTSIZ register value */

    uUsbHccharReg =  USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                            USB_SYNOPSYSHCD_HCCHAR(uChannel));
    uUsbHctsizReg =  USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                            USB_SYNOPSYSHCD_HCTSIZ(uChannel));

    /*
     * Set request's transfer flag to indicate the hardware has
     * already finished the transfer process
     */

    pRequestInfo->uTransferFlag &= ~USB_SYNOPSYSHCD_REQUEST_STILL_TRANSFERING;

    /* Calculating the number of bytes successfully transfered */

    uLengthThisTransfer = usbSynopsysGetTransferLength(pRequestInfo,
                                                       uUsbHctsizReg,
                                                       uUsbHccharReg);
    /*
     * "uLengthThisTransfer < pHCDPipe->uMaximumPacketSize"
     * "((uLengthThisTransfer > pHCDPipe->uMaximumPacketSize) &&
     *   ((uLengthThisTransfer % pHCDPipe->uMaximumPacketSize) != 0))"
     * both means we get a short packet
     *
     * After a series of transaction we will get interrupt.
     * The transfer length may bigger than the max packet size, but
     * if "uLengthThisTransfer % pHCDPipe->uMaximumPacketSize) != 0"
     * it also means we get a short packet at the last stage of the
     * transaction.
     * We will complete the transfer if we get a short packet
     */

    if ((uLengthThisTransfer < pHCDPipe->uMaximumPacketSize) ||
        ((uLengthThisTransfer > pHCDPipe->uMaximumPacketSize) &&
        ((uLengthThisTransfer % pHCDPipe->uMaximumPacketSize) != 0)))
        uEndPacket = 1;

    /*
     * As a special case, setup transactions output the setup header, not
     * the user's data. For this reason we don't count setup data as bytes
     * transfered
     */

    if ((pRequestInfo->uStage == USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP) ||
        (pRequestInfo->uStage == USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP_SPLIT_COMPLETE))
        uLengthThisTransfer = 0;

    /*
     * Add the bytes transfered to the running total. It is important that
     * uLengthThisTransfer doesn't count any data that needs to be
     * retransmitted
     */

    pRequestInfo->uActLength += uLengthThisTransfer;

    if (pHCDPipe->uEndpointType == USB_ATTR_ISOCH)
        {
        if (pRequestInfo->pUrb != NULL)
            {
            pIsoPacketDesc = (pUSBHST_ISO_PACKET_DESC)
                              pRequestInfo->pUrb->pTransferSpecificData;
            if (NULL != pIsoPacketDesc)
                {
                uLengthLeftToTransfer = pIsoPacketDesc->uLength -
                                        pRequestInfo->uActLength;
                }
            }
        }
    else
        {
        if (pRequestInfo->pUrb != NULL)
            {
            uLengthLeftToTransfer = pRequestInfo->pUrb->uTransferLength -
                                    pRequestInfo->uActLength;
            }
        if (uEndPacket)
            uLengthLeftToTransfer = 0;
        }

    /*
     * We need to remember the PID toggle state for the next transaction.
     * The hardware already updated it for the next transaction
     */

    USB_SYNOPSYSHCD_SAVE_DATA_TOGGLE(pHCDData, pHCDPipe, uChannel);

    /*
     * For high speed bulk out, assume the next transaction will need to do a
     * ping before proceeding. If this isn't true the ACK processing below
     * will clear this flag.
     */

    if ((pHCDPipe->uSpeed == USBHST_HIGH_SPEED) &&
        (pHCDPipe->uEndpointType == USB_ATTR_BULK) &&
        (pHCDPipe->uEndpointDir == USB_SYNOPSYSHCD_DIR_OUT))
        pHCDPipe->uPipeFlag |= USB_SYNOPSYSHCD_PIPE_FLAG_NEED_PING;

    /*
     * If the request already be canceld, complete the request with
     * cancel status.
     */

    if (pRequestInfo->uTransferFlag & USB_SYNOPSYSHCD_REQUEST_TO_BE_CANCELED)
        {
        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                          USBHST_TRANSFER_CANCELLED);

        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_STALL)
        {
        /*
         * STALL as a response means this transaction cannot be completed
         * because the device can't process transactions. Complete the
         * request will "USBHST_STALL_ERROR" status.
         */

        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_STALL;
        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                          USBHST_STALL_ERROR);

        }
#if defined(_WRS_CONFIG_ALTERA_SOC_STRATIX10_VP)
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_XFERCOMPL)
        {
        usbSynopsysDwcHcdChannelProcessXFERCOMPLResponse(pHCDData,
                                                         pHCDPipe,
                                                         pRequestInfo,
                                                         uChannel,
                                                         uLengthLeftToTransfer,
                                                         uLengthThisTransfer);
        }
#endif
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_XACTERR)
        {
        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_XACTERR;
        usbSynopsysHcdChannelProcessXACTERRResponse(pHCDData,
                                                    pHCDPipe,
                                                    pRequestInfo,
                                                    uChannel);

        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_BBLERR)
        {
        /* Babble Error */

        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_BBLERR;
        usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                          pHCDPipe,
                                          pRequestInfo,
                                          uChannel,
                                          USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                          USBHST_FAILURE);

        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_FRMOVRUN)
        {
        /* Frame Overrun */

        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_FRMOVRUN;
        usbSynopsysHcdChannelProcessFRMOVRUNResponse(pHCDData,
                                                     pHCDPipe,
                                                     pRequestInfo,
                                                     uChannel);
         }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_NYET)
        {
        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_NYET;
        usbSynopsysHcdChannelProcessNYETResponse(pHCDData,
                                                 pHCDPipe,
                                                 pRequestInfo,
                                                 uChannel,
                                                 uLengthLeftToTransfer,
                                                 uLengthThisTransfer);

        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_ACK)
        {
        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_ACK;
        /*
         * We will always get the ACK and NAK interrupt simultaneously.
         * In this situation, we should process the ACK interrupt
         * and mask the NAK interrupt to forbid the following action.
         * As in the ACK process we already send the following data,
         * the NAK can be ignored.
         */

        if ((uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_NAK))
            {
            uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_NAK;
            }
         usbSynopsysHcdChannelProcessACKResponse(pHCDData,
                                                 pHCDPipe,
                                                 pRequestInfo,
                                                 uChannel,
                                                 uLengthLeftToTransfer,
                                                 uLengthThisTransfer);
        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_NAK)
        {
        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_NAK;
        usbSynopsysHcdChannelProcessNAKResponse(pHCDData,
                                                pHCDPipe,
                                                pRequestInfo);

        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_DATAGLERR)
        {
        /*
         * As we can get ACK&DATAGLERR simultaneously sometimes.
         * The PRI of ACK is higer, we move the DATAGLERR after
         * ACK.
         */

        uUsbHcintReg &= ~USB_SYNOPSYSHCD_HCINT_DATAGLERR;
        usbSynopsysHcdChannelProcessDATAGLERResponse(pHCDData,
                                                     pHCDPipe,
                                                     pRequestInfo,
                                                     uChannel);
        }
    else if (uUsbHcintReg & USB_SYNOPSYSHCD_HCINT_CHHLTD)
        {

        /*
         * We get channel halted interrupts with no result bits sets when the
         * cable is unplugged
         */

        /*
         * Sometimes we find that there do exist halt interrupt
         * with no result bits in bulk transfer. return USBHST_FAILURE directly
         * will wrongly detach the device. So we read the HPRT register
         * to judge whether the device is attached or not. if already detach
         * we will return USBHST_FAILURE directly. if not, we should do nothing
         * just wait for another interrupt
         */

        switch (pHCDPipe->uEndpointType)
            {
            case USB_ATTR_INTERRUPT:
            case USB_ATTR_ISOCH:
                /* Resend the request to avoid delete the device wrongly */

                usbSynopsysHcdChannelProcessNAKResponse(pHCDData,
                                                        pHCDPipe,
                                                        pRequestInfo);

                break;
            case USB_ATTR_CONTROL:
                 uUsbHprtReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                         USB_SYNOPSYSHCD_HPRT);

                 /* Judge whether the device is connect or not */

                 if (uUsbHprtReg & USB_SYNOPSYSHCD_HPRT_PRTCONNSTS)
                    {
                     USB_SHCD_ERR("No result in halt, there must be"
                                  "something wrong with the hardware "
                                  "0x%x, 0x%x, 0x%x, 0x%x.\n",
                                  pHCDPipe->uEndpointType, uUsbHprtReg,
                                  uUsbHcintReg, uChannel, 0, 0);

                     if (pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_ALTERA_SOC_GEN5
                         || pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_RPI_3)
                         {
                         usbSynopsysHcdChannelProcessACKResponse(pHCDData,
                                                                 pHCDPipe,
                                                                 pRequestInfo,
                                                                 uChannel,
                                                                 uLengthLeftToTransfer,
                                                                 uLengthThisTransfer);
                        }
                    }
                 else
                    {
                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      pRequestInfo,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                      USBHST_STALL_ERROR);
                    }

                break;
            case USB_ATTR_BULK:
            default:
                    usbSynopsysHcdFillTaskInfoAndSend(pHCDData,
                                                      pHCDPipe,
                                                      pRequestInfo,
                                                      uChannel,
                                                      USB_SYNOPSYSHCD_TRANSFER_CMD_COMPLITE,
                                                      USBHST_STALL_ERROR);
                break;
            }

        }

    (void) OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

    }

/*******************************************************************************
*
* usbSynopsysHcdPipeNeedSchedule - check if the pipe can be scheduled
*
* This routine checks if the pipe can be scheduled to start the transfer.
*
* RETURNS: TRUE, or FALSE if the pipe cann't be schedule.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL BOOL usbSynopsysHcdPipeNeedSchedule
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe      /* Pointer to the HCDPipe */
    )
    {
    UINT16 uMicroFrameNum = 0x0;
    UINT16 uCurrentFrameNumber = 0x0;
    UINT16 uPipeLastFrameNum = 0x0;
    UINT16 uPipeNextFrameNum = 0x0;
    UINT16 uFrameLowBoundary = 0x0;
    UINT16 uFrameHighBoundary = 0x0;
    BOOL   bSplitInterrupt = TRUE;
    BOOL   bSchedule = FALSE;
    int result = 0x0;

    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo = NULL;


    /* Parameter varification */

    if ((NULL == pHCDData) ||
        (NULL == pHCDPipe))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return FALSE;
        }

    pRequestInfo = usbSynopsysHcdFirstReqGet(pHCDPipe);

    if ((NULL == pRequestInfo)||(NULL == pRequestInfo->pUrb))
        {
        USB_SHCD_ERR("Request not valid, %s is NULL, %d.\n",
                     ((NULL == pRequestInfo) ?
                      "pRequestInfo" :"pRequestInfo->pUrb"),
                     lstCount(&pHCDPipe->reqList),3,4,5,6);

        return FALSE;
        }

    /* If pipe is already scheduled or delete, return FALSE */

    if ((pHCDPipe->uPipeFlag & USB_SYNOPSYSHCD_PIPE_FLAG_DELETE)||
        (pHCDPipe->uPipeFlag & USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED))
        {
        USB_SHCD_DBG("usbSynopsysHcdPipeNeedSchedule - "
                     "Request already delete or scheduled \n",
                     0, 0, 0, 0, 0, 0);
        return FALSE;
        }

    /* If pipe is halted, return FALSE */

    if (pHCDPipe->uHalted > 0)
        {
        pHCDPipe->uHalted--;
        USB_SHCD_WARN("Pipe %p type %d is halted waiting %d SOF for transfer\n",
                      pHCDPipe, pHCDPipe->uEndpointType, pHCDPipe->uHalted, 0, 0, 0);
        return FALSE;
        }

    /* Read the current micro frame number */

    (void) usbSynopsysHcdGetFrameNumber((UINT8)pHCDData->uBusIndex,
                                        &uMicroFrameNum);

    /*
     * For split periodic transfer we should use full frame to schedule
     * except this, we should use micro frame to schedule
     * so we distinguish the related schedule parameter
     * to get the right schedule
     */

    if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe) &&
        (USB_SYNOPSYSHCD_IS_PERIODIC(pHCDPipe)))
        {
        uCurrentFrameNumber = USB_SYNOPSYSHCD_GET_FULL_FRAME_NUMBER(uMicroFrameNum);
        uPipeLastFrameNum = pHCDPipe->uFullFrameNumberLast;
        uPipeNextFrameNum = pHCDPipe->uFullFrameNumberNext;
        uFrameLowBoundary = 0x30;
        uFrameHighBoundary = 0x7CF;

        /*
         * The split interrupt transfer should scheduled by full frame
         * but when (microframe0~microframe7) transfer should start
         * we should take care about it. Now we always let the transfer
         * start after microframe0
         * This will make the split interrupt transfer stably
         */

        if (pHCDPipe->uEndpointType == USB_ATTR_INTERRUPT)
            bSplitInterrupt = ((uMicroFrameNum % 8) == 0);
        }
    else
        {
        uCurrentFrameNumber = uMicroFrameNum;
        uPipeLastFrameNum = pHCDPipe->uMicroFrameNumberLast;
        uPipeNextFrameNum = pHCDPipe->uMicroFrameNumberNext;
        uFrameLowBoundary = 0x300;
        uFrameHighBoundary = 0x3CFF;
        }

    (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER);

    /* 0 <= last <= next <= current */

    if ((uPipeLastFrameNum <= uPipeNextFrameNum) &&
        (uPipeNextFrameNum <= uCurrentFrameNumber) &&
        bSplitInterrupt)
        {
        bSchedule = TRUE;
        }

    /*
     * This judgement covers the condition that the current
     * frame number overflow, the pipe's last/next frame number
     * goes near the max frame number.
     */

    if ((uCurrentFrameNumber <= uFrameLowBoundary) &&
        (uFrameHighBoundary <= uPipeLastFrameNum) &&
        (uPipeLastFrameNum <= uPipeNextFrameNum)&&
        bSplitInterrupt)
        {
        bSchedule = TRUE;
        }

    /*
     * This judgement covers the condition that the current and pipe's next
     * frame number overflow, the pipe's last frame number goes near the max
     * frame number.
     */

    if ((uPipeNextFrameNum <= uCurrentFrameNumber) &&
        (uCurrentFrameNumber <= uFrameLowBoundary) &&
        (uFrameHighBoundary <= uPipeLastFrameNum)&&
        bSplitInterrupt)
        {
        bSchedule = TRUE;
        }

    if (bSchedule)
        {
        /*
         * Move Set uPipeFlag here, in SOF interrupt handler this entry
         * can be re-enter, and the data may be send twice. This can
         * raise timeout error.
         */

        pHCDPipe->uPipeFlag |= USB_SYNOPSYSHCD_PIPE_FLAG_SCHEDULED;
        pHCDPipe->uMicroFrameNumberLast = uMicroFrameNum;
        pHCDPipe->uMicroFrameNumberNext = (pHCDPipe->uMicroFrameNumberLast
                                           + pHCDPipe->bInterval) %
                                           USB_SYNOPSYSHCD_FRAME_NUM_MAX;
        /* Store the full frame number */

        pHCDPipe->uFullFrameNumberLast =
        USB_SYNOPSYSHCD_GET_FULL_FRAME_NUMBER(pHCDPipe->uMicroFrameNumberLast);
        pHCDPipe->uFullFrameNumberNext = (pHCDPipe->uFullFrameNumberLast +
                                          pHCDPipe->bInterval)%
                                          USB_SYNOPSYSHCD_FULL_FRAME_NUM_MAX;

        result = usbSynopsysHcdMoveRequetToReadyList(pHCDData,
                                            pHCDPipe,
                                            pRequestInfo);
        (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

        return (result == OK);
        }

    (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
    return FALSE;
    }

/*******************************************************************************
*
* usbSynopsysHcdCheckSchedule - check pipe list to find a pipe can be scheduled
*
* This routine checks the pipe list to find a pipe can be scheduled.
*
* RETURNS: pointer to the pipe can be schedule or NULL
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL pUSB_SYNOPSYSHCD_PIPE usbSynopsysHcdCheckSchedule
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe      /* Pointer to the HCDPipe */
    )
    {

    if ((NULL == pHCDPipe)||(NULL == pHCDData))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return NULL;
        }

    if (usbSynopsysHcdPipeNeedSchedule(pHCDData, pHCDPipe))
        return pHCDPipe;
    else
        return NULL;
    }

/*******************************************************************************
*
* usbSynopsysHcdControlChannelStart - start to process the control channel
*
* This routine starts to process the control channel.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdControlChannelStart
    (
    pUSB_SYNOPSYSHCD_DATA   pHCDData,            /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE   pHCDPipe,            /* Pointer to the HCDPipe */
    UINT8                   uChannel,            /* DMA transfer channel */
    UINT32                  uLengthToTransfer    /* Length intend to transfer */
    )
    {
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo = NULL;
    pUSBHST_SETUP_PACKET pSetup = NULL;
    UINT32 uUsbHctsizReg = 0;
    UINT32 uUsbHcspltReg = 0;
    UINT32 uUsbHccharReg = 0;
    VIRT_ADDR dmaAddress = 0;
    PHYS_ADDR dmaPhyAddr = 0;

    /* Parameter varification */

    if ((NULL == pHCDData) ||
        (NULL == pHCDPipe))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /* Find one request in the pipe queue, if no request available, return */

    pRequestInfo = usbSynopsysHcdFirstReqGet(pHCDPipe);

    if ((NULL == pRequestInfo) || (NULL == pRequestInfo->pUrb))
        {
        USB_SHCD_ERR("RequestInfo not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    pSetup = (pUSBHST_SETUP_PACKET)pRequestInfo->pUrb->pTransferSpecificData;

    uUsbHctsizReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_HCTSIZ(uChannel));

    /* Set packet count to 1 */

    uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PKTCNT);
    uUsbHctsizReg |= (0x1 << USB_SYNOPSYSHCD_HCTSIZ_PKTCNT_OFFSET);

    switch (pRequestInfo->uStage)
        {
        case USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL:
        case USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE:
            USB_SHCD_ERR("Conrol RequestInfo Stage is wrong \n",
                         0, 0, 0, 0, 0, 0);
            break;
        case USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP:

            /* Set pid */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PID);
            uUsbHctsizReg |= (0x3 << USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET);

            /* Set transfer size */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);
            uUsbHctsizReg |= (sizeof(* pSetup) <<
                              USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);

            /* All Control operations start with a setup going OUT */

            uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                            USB_SYNOPSYSHCD_HCCHAR(uChannel));
            /* Set EP direction */

            uUsbHccharReg &= ~(USB_SYNOPSYSHCD_HCCHAR_EPDIR);

            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                        uUsbHccharReg);

            /* "pSetup" points to the send buffer. Use this to updata the DMA */

            /*
             * VxWorks 64 bit notes : It is expected that DMA transfer buffers
             * are from DMA32 spacce, which is in the lower 4GB of the 64 bit
             * memory space. So the casting to UINT32 is safe here.
             */

            dmaAddress = (VIRT_ADDR) pSetup;
            if (dmaAddress != 0)
                {
                CACHE_USER_FLUSH(dmaAddress, sizeof(* pSetup));
                CACHE_USER_INVALIDATE(dmaAddress, sizeof(* pSetup));

#if CPU==MIPSI64R2
                USB_SYNOPSYSHCD_PROGRAM_DMA_ADDRESS(pHCDData,
                    USB_SYNOPSYSHCD_DMA0_OUTB_CHN(uChannel),
                    USB_SYNOPSYSHCD_CONVERT_TO_BUS_MEM(pHCDData->uBusIndex,
                                                       dmaAddress));
#else
                dmaPhyAddr = usbSynopsysHcdVirToPhy(dmaAddress);

                /* For Raspberry Pi 3, software accessing RAM using the
                 * DMA engines must use bus address (based at 0xC0000000)
                 */

                if (pHCDData->uPlatformType 
                    == USB_SYNOPSYSHCI_PLATFORM_RPI_3)
                    {
                    dmaPhyAddr = 0xC0000000 | (UINT32) dmaPhyAddr;
                    }
                    
                USB_SYNOPSYSHCD_PROGRAM_DMA_ADDRESS(pHCDData,
                    USB_SYNOPSYSHCD_HCDMA(uChannel),
                    dmaPhyAddr);
#endif
                }
            break;
        case USB_SYNOPSYSHCD_REQUEST_STAGE_SETUP_SPLIT_COMPLETE:

            /* Set pid */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PID);
            uUsbHctsizReg |= (0x3 << USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET);

            /* Set Transfer size to 0 */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);
            uUsbHctsizReg |= (0 <<
                              USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);


            /* All Control operations start with a setup going OUT */

            uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCCHAR(uChannel));
            /* Set EP direction */

            uUsbHccharReg &= ~(USB_SYNOPSYSHCD_HCCHAR_EPDIR);
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                        uUsbHccharReg);

            /* HCSPLT register set do complete 1 */

            uUsbHcspltReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCSPLT(uChannel));

            uUsbHcspltReg &= ~(USB_SYNOPSYSHCD_HCSPLT_COMPSPLT);
            uUsbHcspltReg |= (0x1 << USB_SYNOPSYSHCD_HCSPLT_COMPSPLT_OFFSET);

            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCSPLT(uChannel),
                                        uUsbHcspltReg);


            break;
        case USB_SYNOPSYSHCD_REQUEST_STAGE_DATA:

            /* Set pid */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PID);
            uUsbHctsizReg |= (UINT32)(USB_SYNOPSYSHCD_GET_DATA_TOGGLE(pHCDPipe) <<
                                      USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET) &
                                     USB_SYNOPSYSHCD_HCTSIZ_PID;

            /* Set transfer size */

            /*
             * No matter split or not the HCTSIZE must be setted
             * Or else we woun't get the right data
             * This will cause exception when the Haier keyboard+mouse
             * plug in
             */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);


            if ((pSetup->bmRequestType & USB_SYNOPSYSHCD_DIR_IN) == 0)
                {
                /*
                 * At data stage of the control transfer
                 * we donn't alwasys need to set the transfer size equal to the
                 * pSetup->wLength. In this stage we may get NAK, we should
                 * set the HCTSIZ as the real transfer size we intend to do.
                 */

                if (uLengthToTransfer < (OS_UINT16_LE_TO_CPU(pSetup->wLength)))
                    {
                    uUsbHctsizReg |= (uLengthToTransfer <<
                                      USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);
                    }
                else
                    {
                    uUsbHctsizReg |= (UINT32)(OS_UINT16_LE_TO_CPU(pSetup->wLength) <<
                                              USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);
                    }
                }
            else
                {
                uUsbHctsizReg |= (uLengthToTransfer <<
                          USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);

                }

            /* Set EP direction */

            uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_HCCHAR(uChannel));

            uUsbHccharReg &= ~(USB_SYNOPSYSHCD_HCCHAR_EPDIR);

            if (pSetup->bmRequestType & USB_SYNOPSYSHCD_DIR_IN)
                {
                uUsbHccharReg |=  (USB_SYNOPSYSHCD_HCCHAR_EPDIR);
                }
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                        uUsbHccharReg);


            break;
        case USB_SYNOPSYSHCD_REQUEST_STAGE_DATA_SPLIT_COMPLETE:

            /* Set pid */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PID);
            uUsbHctsizReg |= (UINT32)(USB_SYNOPSYSHCD_GET_DATA_TOGGLE(pHCDPipe) <<
                              USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET) &
                              USB_SYNOPSYSHCD_HCTSIZ_PID;

            /* Set transfer size */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);
            if (pSetup->bmRequestType & USB_SYNOPSYSHCD_DIR_IN)
                {
                uUsbHctsizReg |= (uLengthToTransfer <<
                          USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);
                }

            /* Set EP direction */

            uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_HCCHAR(uChannel));
            uUsbHccharReg &= ~(USB_SYNOPSYSHCD_HCCHAR_EPDIR);

            if (pSetup->bmRequestType & USB_SYNOPSYSHCD_DIR_IN)
                {
                uUsbHccharReg |=  (USB_SYNOPSYSHCD_HCCHAR_EPDIR);
                }
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                        uUsbHccharReg);
            /* Do Split */

            uUsbHcspltReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_HCSPLT(uChannel));

            uUsbHcspltReg &= ~(USB_SYNOPSYSHCD_HCSPLT_COMPSPLT);
            uUsbHcspltReg |= (0x1 << USB_SYNOPSYSHCD_HCSPLT_COMPSPLT_OFFSET);

            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCSPLT(uChannel),
                                        uUsbHcspltReg);


            break;
        case USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS:

            /* Set pid */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PID);
            uUsbHctsizReg |= (UINT32)(USB_SYNOPSYSHCD_GET_DATA_TOGGLE(pHCDPipe) <<
                                      USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET) &
                                     USB_SYNOPSYSHCD_HCTSIZ_PID;

            /* Set transfer size to 0 */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);

            /* Set EP direction */

            uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCCHAR(uChannel));

            if (pSetup->bmRequestType & USB_SYNOPSYSHCD_DIR_IN)
                {
                uUsbHccharReg &= ~(USB_SYNOPSYSHCD_HCCHAR_EPDIR);
                }
            else
                {
                uUsbHccharReg |=  (USB_SYNOPSYSHCD_HCCHAR_EPDIR);
                }
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                        uUsbHccharReg);


            break;
        case USB_SYNOPSYSHCD_REQUEST_STAGE_STATUS_SPLIT_COMPLETE:

            /* Set pid */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_PID);
            uUsbHctsizReg |= (UINT32)(USB_SYNOPSYSHCD_GET_DATA_TOGGLE(pHCDPipe) <<
                                      USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET) &
                                     USB_SYNOPSYSHCD_HCTSIZ_PID;

            /* Set transfer size to 0 */

            uUsbHctsizReg &= ~(USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);

            /* Set EP direction */

            uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCCHAR(uChannel));
            if (pSetup->bmRequestType & USB_SYNOPSYSHCD_DIR_IN)
                {
                uUsbHccharReg &= ~(USB_SYNOPSYSHCD_HCCHAR_EPDIR);
                }
            else
                {
                uUsbHccharReg |=  (USB_SYNOPSYSHCD_HCCHAR_EPDIR);
                }
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                        uUsbHccharReg);
            /* Do Split */

            uUsbHcspltReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCSPLT(uChannel));

            uUsbHcspltReg &= ~(USB_SYNOPSYSHCD_HCSPLT_COMPSPLT);
            uUsbHcspltReg |= (0x1 << USB_SYNOPSYSHCD_HCSPLT_COMPSPLT_OFFSET);

            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HCSPLT(uChannel),
                                        uUsbHcspltReg);

            break;
        }
    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCTSIZ(uChannel),
                                uUsbHctsizReg);

    return;
    }

/*******************************************************************************
*
* usbSynopsysProgramHCCHAR - Program the HCCHAR register
*
* This routine programes the HCCHAR register.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL __inline__ VOID usbSynopsysProgramHCCHAR
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,     /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo, /* Pointer to the request */
    UINT8                         uChannel      /* DMA transfer channel */
    )
    {
    UINT32 uUsbHccharReg = 0x0;
    UINT32 uUsbFrnumReg = 0x0;
    UINT32 uTemp = 0x0;

    /*
     * This function wount't check the valid of the parameters
     * The caller function should ensure it
     */

    /* Program the maximum packet size field */

    uTemp = (UINT32)(pHCDPipe->uMaximumPacketSize <<
                     USB_SYNOPSYSHCD_HCCHAR_MPS_OFFSET) &
                     USB_SYNOPSYSHCD_HCCHAR_MPS;
    uUsbHccharReg |= uTemp;

    /* Program the endpoint number field */

    uTemp = (UINT32)(pHCDPipe->uEndpointAddress <<
                     USB_SYNOPSYSHCD_HCCHAR_EPNUM_OFFSET) &
                     USB_SYNOPSYSHCD_HCCHAR_EPNUM;
    uUsbHccharReg |= uTemp;

    /* Program the endpoint direction field */

    uTemp = (UINT32)(pHCDPipe->uEndpointDir <<
                     USB_SYNOPSYSHCD_HCCHAR_EPDIR_OFFSET) &
                     USB_SYNOPSYSHCD_HCCHAR_EPDIR;
    uUsbHccharReg |= uTemp;

    /* Program the low-speed device field */

    if (pHCDPipe->uSpeed == USBHST_LOW_SPEED)
        {
        uTemp = USB_SYNOPSYSHCD_HCCHAR_LSPDDEV;
        uUsbHccharReg |= uTemp;
        }

    /* Program the endpoint type field */

    uTemp = (UINT32)(pHCDPipe->uEndpointType <<
                     USB_SYNOPSYSHCD_HCCHAR_EPTYPE_OFFSET) &
                     USB_SYNOPSYSHCD_HCCHAR_EPTYPE;
    uUsbHccharReg |= uTemp;

    /* Program the multicount/error count field */

    uTemp = ((pHCDPipe->uMaximumPacketSize &
            USB_SYNOPSYSHCD_ENDPOINT_NUMBER_OF_TRANSACTIONS_MASK) >> 11 ) + 1;

    pHCDPipe->uMultiTransPerMicroFrame = (uTemp & 0xFFFF);

    if (uTemp > 3)
        {
        uUsbHccharReg |= (0x3 << USB_SYNOPSYSHCD_HCCHAR_EC_OFFSET);
        }
    else
        {
        uUsbHccharReg |= (uTemp << USB_SYNOPSYSHCD_HCCHAR_EC_OFFSET);
        }

    /* Program the device address field */

    uTemp = (UINT32)(pHCDPipe->uDeviceAddress <<
                     USB_SYNOPSYSHCD_HCCHAR_DEVADDR_OFFSET) &
                     USB_SYNOPSYSHCD_HCCHAR_DEVADDR;
    uUsbHccharReg |= uTemp;

    /* Only periodic transfer need program the ODDFRM field */

    if (USB_SYNOPSYSHCD_IS_PERIODIC(pHCDPipe))
        {
        uUsbFrnumReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                               USB_SYNOPSYSHCD_HFNUM);
        /*
         * To delay one micro frame then send the complit interrupt split
         * This action will reduce the NYET times
         * let hub NAK if no data here
         */

        if ((pHCDPipe->uEndpointType == USB_ATTR_INTERRUPT) &&
            USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe) &&
            (pRequestInfo->uStage ==
            USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE))
            {
            uUsbFrnumReg = (uUsbFrnumReg + 1) % USB_SYNOPSYSHCD_FRAME_NUM_MAX;
            }

        if (uUsbFrnumReg & 1)
            {
            uUsbHccharReg |= USB_SYNOPSYSHCD_HCCHAR_ODDFRM;
            }
        }

    /* Write the new value to the HCCHAR register */

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                uUsbHccharReg);
    return;
    }

/*******************************************************************************
*
* usbSynopsysProgramChannelInterrupt - program the interrupt register
*
* This routine programs the interupt register to prepare for the transfer.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL __inline__ VOID usbSynopsysProgramChannelInterrupt
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,     /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,     /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo, /* Pointer to the request */
    UINT8                         uChannel      /* DMA transfer channel */
    )
    {
    UINT32 uTemp = 0x0;
    UINT32 uInterruptMask     = 0;

    /* Clear all channel status bits */

    uTemp = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                       USB_SYNOPSYSHCD_HCINT(uChannel));

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCINT(uChannel),
                                uTemp);

    /* Enable the channel halt interrupt */

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCINTMSK(uChannel),
                                (USB_SYNOPSYSHCD_HCINTMSK_CHHLTDMSK));

    /* Enable the channel interrupt to propagate */

    uTemp = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                       USB_SYNOPSYSHCD_HAINTMSK);

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HAINTMSK,
                                (uTemp | (0x1 << uChannel)));

    uInterruptMask = USB_SYNOPSYSHCD_READ32_REG (pHCDData,
                                               USB_SYNOPSYSHCD_GINTMSK);
    USB_SYNOPSYSHCD_WRITE32_REG (pHCDData ,
                                 USB_SYNOPSYSHCD_GINTMSK,
                                 uInterruptMask |
                                 USB_SYNOPSYSHCD_GINTSTS_HCHINT );
    return;
    }

/*******************************************************************************
*
* usbSynopsysProgramHCTSIZ - Program the HCTSIZ register
*
* This routine programs the HCTSIZ register to prepare for the transfer.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL __inline__ VOID usbSynopsysProgramHCTSIZ
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,         /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,         /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,     /* Pointer to the request */
    UINT8                         uChannel,         /* DMA transfer channel */
    UINT32                        uLengthToTransfer /* Length intend to transfer */
    )
    {
    UINT32 uTemp = 0x0;
    UINT32 uUsbHctsizReg = 0x0;

    /* Get the packet count need to transfer */

    uTemp = (uLengthToTransfer + pHCDPipe->uMaximumPacketSize - 1) /
            pHCDPipe->uMaximumPacketSize;

    /* If less than 1,just set it to 1 */

    if (uTemp < 1)
        uTemp = 1;

    /* Adjust the PKTCCNT in proper register field */

    uTemp = (uTemp << USB_SYNOPSYSHCD_HCTSIZ_PKTCNT_OFFSET) &
            USB_SYNOPSYSHCD_HCTSIZ_PKTCNT ;

    /* Fill the the transfer length and packet count */

    uUsbHctsizReg = uTemp |
          (uLengthToTransfer & USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE);

    /* Fill the PID data */

    uTemp = (UINT32)USB_SYNOPSYSHCD_GET_DATA_TOGGLE(pHCDPipe);

    uTemp = (uTemp << USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET) &
             USB_SYNOPSYSHCD_HCTSIZ_PID;

    uUsbHctsizReg |= uTemp;

    /* Fill the do ping bit */

    if (pHCDPipe->uPipeFlag & USB_SYNOPSYSHCD_PIPE_FLAG_NEED_PING)
        {
        uTemp = USB_SYNOPSYSHCD_HCTSIZ_DOPING;
        uUsbHctsizReg |= uTemp;
        }

    if (pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_ALTERA_SOC_GEN5
        || pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_RPI_3)
        {
        if ((pHCDPipe->uSpeed == USBHST_HIGH_SPEED) &&
            (pHCDPipe->uEndpointType == USB_ATTR_BULK) &&
            (pHCDPipe->uEndpointDir == USB_SYNOPSYSHCD_DIR_OUT))
            {
            uTemp = USB_SYNOPSYSHCD_HCTSIZ_DOPING;
            uUsbHctsizReg |= uTemp;
            }
        }

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCTSIZ(uChannel),
                                uUsbHctsizReg);
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdChannelStartProcess - start process the channel
*
* This routine starts process the channel,such as transfering, receiving, etc.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdChannelStartProcess
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,  /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,  /* Pointer to the HCDPipe */
    UINT8                         uChannel   /* DMA transfer channel */
    )
    {
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo = NULL;
    pUSBHST_ISO_PACKET_DESC pIsoPacketDesc = NULL;
    UINT32 uUsbHcspltReg = 0x0;
    UINT32 uLengthToTransfer  = 0x0;
    VIRT_ADDR dmaAddress = 0x0;
    PHYS_ADDR dmaPhyAddr = 0x0;
    UINT32 uTemp = 0x0;
    UINT8  uHubAddr = 0x0;
    UINT8  uPrtAddr = 0x0;

    /* Parameter varification */

    if ((NULL == pHCDData) || (NULL == pHCDPipe))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /* Find one request in the pipe queue, if no request available, return */

    pRequestInfo = usbSynopsysHcdFirstReqGet(pHCDPipe);

    if ((NULL == pRequestInfo)||(NULL == pRequestInfo->pUrb))
        {
        USB_SHCD_ERR("Request not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }
    /* Clear the bit to indentify the channel is using */

    pHCDPipe->uDmaChannel = uChannel;
    pRequestInfo->uTransferFlag |= USB_SYNOPSYSHCD_REQUEST_STILL_TRANSFERING;
    pHCDData->pPipeInChannel[uChannel] = pHCDPipe;

    /* Prgrom the related interrupt to prepare for the transfer */

    usbSynopsysProgramChannelInterrupt(pHCDData,
                                       pHCDPipe,
                                       pRequestInfo,
                                       uChannel);

    /* Setup the locations the DMA engines use */

    {
    dmaAddress = (VIRT_ADDR) (pRequestInfo->pUrb->pTransferBuffer +
                              pRequestInfo->uActLength);

    uLengthToTransfer = pRequestInfo->pUrb->uTransferLength -
                        pRequestInfo->uActLength;

    /*
     * ISO transfer use ISO packet descriptor to indicate the transfer buffer
     * and the transfer size, So we should treat the ISO transfer in different
     * way.
     */

    /* Get the first ISO packet descriptor */

    pIsoPacketDesc = (pUSBHST_ISO_PACKET_DESC)
                     pRequestInfo->pUrb->pTransferSpecificData;

    /* Get the current ISO packet descriptor */

    pIsoPacketDesc =
        &pIsoPacketDesc[pRequestInfo->uUrbTotalPacketCount -    \
                        pRequestInfo->pUrb->uNumberOfPackets];

    /* Get right transfer buffer and size from the ISO packet descriptor */

    if ((pHCDPipe->uEndpointType == USB_ATTR_ISOCH) &&
        (NULL != pIsoPacketDesc))
        {
        dmaAddress = (VIRT_ADDR) (pRequestInfo->pUrb->pTransferBuffer +
                                  pIsoPacketDesc->uOffset +
                                  pRequestInfo->uActLength);
        uLengthToTransfer = pIsoPacketDesc->uLength - pRequestInfo->uActLength;
        }

    if (dmaAddress != NULL)
        {
        CACHE_USER_FLUSH(dmaAddress, uLengthToTransfer);
        CACHE_USER_INVALIDATE(dmaAddress, uLengthToTransfer);

        /* Set DMA transfer buffer address */

#if CPU==MIPSI64R2
        USB_SYNOPSYSHCD_PROGRAM_DMA_ADDRESS(pHCDData,
            USB_SYNOPSYSHCD_DMA0_OUTB_CHN(uChannel),
            USB_SYNOPSYSHCD_CONVERT_TO_BUS_MEM(pHCDData->uBusIndex,
                                               dmaAddress));
        USB_SYNOPSYSHCD_PROGRAM_DMA_ADDRESS(pHCDData,
            USB_SYNOPSYSHCD_DMA0_INB_CHN(uChannel),
            USB_SYNOPSYSHCD_CONVERT_TO_BUS_MEM(pHCDData->uBusIndex,
                                               dmaAddress));
#else
        dmaPhyAddr = usbSynopsysHcdVirToPhy(dmaAddress);

        /* For Raspberry Pi 3, software accessing RAM using the
         * DMA engines must use bus addresses (based at 0xC0000000)
         */

        if (pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_RPI_3)
            {
            dmaPhyAddr = 0xC0000000 | (UINT32) dmaPhyAddr;
            }
            
        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, 
                                    USB_SYNOPSYSHCD_HCDMA(uChannel), 
                                    dmaPhyAddr);
#endif
        }
    }

    /* Setup both the size of the transfer and the SPLIT characteristics */

    {
    /*
     * We need to do split transactions if the device with low speed
     * behind a high speed hub
     */

    if (USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
        {
        /* Set hub address and port address */

        uHubAddr = (pHCDPipe->uHubInfo & 0x7F00) >> 8;
        uPrtAddr = (pHCDPipe->uHubInfo & 0x007F);

        uUsbHcspltReg = (USB_SYNOPSYSHCD_HCSPLT_SPLTENA) |
                 (UINT32)(uHubAddr << USB_SYNOPSYSHCD_HCSPLT_HUBADDR_OFFSET) |
                 (UINT32)(uPrtAddr << USB_SYNOPSYSHCD_HCSPLT_PRTADDR_OFFSET);

        if (pRequestInfo->uStage ==
            USB_SYNOPSYSHCD_REQUEST_STAGE_NON_CONTROL_SPLIT_COMPLETE)
            {
            uUsbHcspltReg |= (0x1 << USB_SYNOPSYSHCD_HCSPLT_COMPSPLT_OFFSET);

            }

        /* Limit the transfer size to the max packet size */

        if (uLengthToTransfer > pHCDPipe->uMaximumPacketSize)
            uLengthToTransfer = pHCDPipe->uMaximumPacketSize;

        /*
         * ISOCHRONOUS OUT splits are unique in that they limit
         * data transfers to 188 byte chunks representing the
         * begin/middle/end of the data or all
         */

        if (!(uUsbHcspltReg & (0x1 <<USB_SYNOPSYSHCD_HCSPLT_COMPSPLT_OFFSET)) &&
             (pHCDPipe->uEndpointDir == USB_SYNOPSYSHCD_DIR_OUT) &&
             (pHCDPipe->uEndpointType == USB_ATTR_ISOCH))
            {

            /* Check if the transfe start or not, update the transfer positon */

            if (pRequestInfo->uActLength == 0)
                {

                /*
                 * Nothing sent yet, this is either a begin or the
                 * entire payload
                 */

                if (uLengthToTransfer <= 188)
                    {
                    /* All */

                    uUsbHcspltReg |= (0x3 <<
                                      USB_SYNOPSYSHCD_HCSPLT_XACTPOS_OFFSET);
                    }
                else
                    {
                    /* Begin */

                    uUsbHcspltReg |= (0x2 <<
                                      USB_SYNOPSYSHCD_HCSPLT_XACTPOS_OFFSET);
                    }
                }
            else
                {
                /* In transfer, the position must be midlle or end */

                if (uLengthToTransfer <= 188)
                    {
                    /* End */

                    uUsbHcspltReg |= (0x1 <<
                                      USB_SYNOPSYSHCD_HCSPLT_XACTPOS_OFFSET);
                    }
                else
                    {
                    /* Middle */

                    uUsbHcspltReg |= (0x0 <<
                                      USB_SYNOPSYSHCD_HCSPLT_XACTPOS_OFFSET);
                    }
                }

            /* The transfer size should limited to 188 bytes */

            if (uLengthToTransfer > 188)
                uLengthToTransfer = 188;
            }

        }

    /* Set the HCSPLT register */

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCSPLT(uChannel),
                                uUsbHcspltReg);

    /* Program the HCTSIZ register */

    usbSynopsysProgramHCTSIZ(pHCDData,
                             pHCDPipe,
                             pRequestInfo,
                             uChannel,
                             uLengthToTransfer);
    }

    /* Setup the Host Channel Characteristics Register */

    usbSynopsysProgramHCCHAR(pHCDData,
                             pHCDPipe,
                             pRequestInfo,
                             uChannel);

    /* Do transaction type specific fixups as needed */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_CONTROL:
            usbSynopsysHcdControlChannelStart(pHCDData,
                                              pHCDPipe,
                                              uChannel,
                                              uLengthToTransfer);

            break;
        case USB_ATTR_BULK:
        case USB_ATTR_INTERRUPT:
            break;
        case USB_ATTR_ISOCH:
            if (!USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData, pHCDPipe))
                {
                /*
                 * ISO transactions require differnet PIDs depending on
                 * direction and how many packets are needed
                 */

                if (pHCDPipe->uEndpointDir == USB_SYNOPSYSHCD_DIR_OUT)
                    {
                    if (pHCDPipe->uMultiTransPerMicroFrame < 2)
                        {
                        /* Need DATA0 */

                        uTemp = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                              USB_SYNOPSYSHCD_HCTSIZ(uChannel));
                        uTemp &= ~USB_SYNOPSYSHCD_HCTSIZ_PID;

                        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                          USB_SYNOPSYSHCD_HCTSIZ(uChannel),
                                          uTemp);

                        }
                    else
                        {
                        /* Need MDATA */

                        uTemp = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                              USB_SYNOPSYSHCD_HCTSIZ(uChannel));
                        uTemp &= ~USB_SYNOPSYSHCD_HCTSIZ_PID;
                        uTemp |= (0x3 << USB_SYNOPSYSHCD_HCTSIZ_PID_OFFSET) &
                                  USB_SYNOPSYSHCD_HCTSIZ_PID;

                        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                          USB_SYNOPSYSHCD_HCTSIZ(uChannel),
                                          uTemp);
                        }
                    }
                }
            break;
        }

    /* Remember the transfer size and packet count in this transfer */

    uTemp = USB_SYNOPSYSHCD_READ32_REG(pHCDData,USB_SYNOPSYSHCD_HCTSIZ(uChannel));

    /*
     * Routine usbSynopsysHcdControlChannelStart will modify the HCTSIZ
     * register and set the USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE field to 0
     * under some conditions. But we should always set the transfer size
     * equal to the length that we intent to transfer.
     */

    pRequestInfo->uXferSize = uLengthToTransfer;

    /*
     * Routine usbSynopsysHcdControlChannelStart will update HCTSIZ register
     * So we move the set of pRequestInfo->uPktcnt from entry
     * usbSynopsysProgramHCTSIZ to here.
     */

    pRequestInfo->uPktcnt = (uTemp & USB_SYNOPSYSHCD_HCTSIZ_PKTCNT) >>
                             USB_SYNOPSYSHCD_HCTSIZ_PKTCNT_OFFSET;

    /*
     * Halt the channel may effect the transfer
     * For bulk/control in transfer, if the target device doesn't respond,
     * the host controller will try to restart the transfer at every SOF.
     * This action may effect others that want to transfer at the start of the SOF.
     * We halt the channel and let it continue transfer once other channel get
     * any response or reach the limit.
     */

    if (pHCDData->uPlatformType != USB_SYNOPSYSHCI_PLATFORM_ALTERA_SOC_GEN5
        && pHCDData->uPlatformType != USB_SYNOPSYSHCI_PLATFORM_RPI_3)
        {
        usbSynopsysHcdHaltChannel(pHCDData, pHCDPipe);
        }

    /* Enable the channel to start the transfer */

    USB_SYNOPSYSHCD_SETBITS32_REG(pHCDData,
                                  USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                  USB_SYNOPSYSHCD_HCCHAR_CHENA);
    }

/*******************************************************************************
*
* usbSynopsysHcdVirToPhy - translate a virtual address to a physical address
*
* This routine retrieves mapping information for a virtual address from
*  the page translation tables.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL PHYS_ADDR usbSynopsysHcdVirToPhy
    (
    VIRT_ADDR virtAdrs
    )
    {
    PHYS_ADDR physAddr;
    (void) vmTranslate (NULL, virtAdrs, &physAddr);
    return physAddr;
    }

/* End of file */
