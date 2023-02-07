/* usbSynopsysHcdUtil_rpi_3.c - utility functions for Synopsys HCD */

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

This module defines the functions which serve as utility functions for the
Synopsys Host Controller Driver.

INCLUDE FILES: usb/usb.h,
               usbSynopsysHcdDataStructures_rpi_3.h,
               usbSynopsysHcdRegisterInfo_rpi_3.h,
               usbSynopsysHcdHardwareAccess_rpi_3.h,
               usbSynopsysHcdUtil_rpi_3.h,
*/

/* includes */

#include <usb.h>
#include "usbSynopsysHcdDataStructures_rpi_3.h"
#include "usbSynopsysHcdRegisterInfo_rpi_3.h"
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"
#include "usbSynopsysHcdUtil_rpi_3.h"

/*******************************************************************************
*
* usbSynopsysGetTransferLength - get the current transfer length
*
* This routine gets the current tranfer length.
*
* RETURNS: 32-bit value of the transfer length.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

UINT32 usbSynopsysGetTransferLength
    (
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo, /* Pointer to the request */
    UINT32                        uUsbHctSizeReg, /* Status of HCTSIZ register */
    UINT32                        uUsbHccharReg   /* Status of HCCHAR register */
    )
    {
    UINT32 uTransferLength = 0x0;
    UINT32 uTemp = 0x0;

    if ((NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter is not valid\n", 0, 0, 0, 0, 0, 0);
        return 0;
        }

    if (uUsbHccharReg & USB_SYNOPSYSHCD_HCCHAR_EPDIR)
        {
        uTransferLength = pRequestInfo->uXferSize -
                          ((uUsbHctSizeReg & USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE) >>
                          USB_SYNOPSYSHCD_HCTSIZ_XFERSIZE_OFFSET);
        }
    else
        {
        uTemp = pRequestInfo->uPktcnt -
               ((uUsbHctSizeReg & USB_SYNOPSYSHCD_HCTSIZ_PKTCNT) >>
               USB_SYNOPSYSHCD_HCTSIZ_PKTCNT_OFFSET);
        uTransferLength = uTemp *
                          ((uUsbHccharReg & USB_SYNOPSYSHCD_HCCHAR_MPS) >>
                          USB_SYNOPSYSHCD_HCCHAR_MPS_OFFSET);

        }

    /* The transfer length should limit to the transer size */

    if (uTransferLength > pRequestInfo->uXferSize)
        uTransferLength = pRequestInfo->uXferSize;
    return uTransferLength;
    }

/*******************************************************************************
*
* usbSynopysHcdDeleteRequestFromScheduleList - delete request from schedule list
*
* This routine deletes the request from the related schedule list.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopysHcdDeleteRequestFromScheduleList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,    /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo /* Pointer to the request */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /*
     * Delete request from related list
     * This function always called by free entry
     * These entry should make sure the synchronization of the list
     */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_CONTROL:
        case USB_ATTR_BULK:
            if (ERROR != lstFind(&(pHCDData->nonPeriodicReqList),
                                &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->nonPeriodicReqList),
                          &(pRequestInfo->schedNode));
                }
            else if (ERROR !=lstFind(&(pHCDData->nonPeriodicReqReadyList),
                                    &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->nonPeriodicReqReadyList),
                          &(pRequestInfo->schedNode));
                }
            break;
        case USB_ATTR_INTERRUPT:
        case USB_ATTR_ISOCH:
            if (ERROR != lstFind(&(pHCDData->periodicReqList),
                                &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->periodicReqList),
                          &(pRequestInfo->schedNode));
                }
            else if (ERROR != lstFind(&(pHCDData->periodicReqReadyList),
                                     &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->periodicReqReadyList),
                          &(pRequestInfo->schedNode));
                }
            break;
        default:
            break;
        }

    return;
}

/*******************************************************************************
*
* usbSynopysHcdDeleteRequestFromReadyList - delete request from ready list
*
* This routine deletes the request from ready list.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopysHcdDeleteRequestFromReadyList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,    /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo /* Pointer to the request */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /*
     * Delete request from Ready list
     * This function always called by free entry,
     * These enry shold make sure the synchronization of the list
     */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_CONTROL:
        case USB_ATTR_BULK:
            if (ERROR != lstFind(&(pHCDData->nonPeriodicReqReadyList),
                                &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->nonPeriodicReqReadyList),
                          &(pRequestInfo->schedNode));
                }
            break;
        case USB_ATTR_INTERRUPT:
        case USB_ATTR_ISOCH:
             if (ERROR != lstFind(&(pHCDData->periodicReqReadyList),
                                 &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->periodicReqReadyList),
                          &(pRequestInfo->schedNode));
                }
            break;
        default:
            break;
        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdAddRequestIntoSendList - add request to sending list
*
* This routine adds the request into sending list.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdAddRequestIntoSendList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,    /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo /* Pointer to the request */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /*
     * Add request into Send list
     * The caller entry shold make sure the synchronization of the list
     */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_CONTROL:
        case USB_ATTR_BULK:
            lstAdd(&(pHCDData->nonPeriodicReqList),
                   &(pRequestInfo->schedNode));
            break;
        case USB_ATTR_INTERRUPT:
        case USB_ATTR_ISOCH:
            lstAdd(&(pHCDData->periodicReqList),
                   &(pRequestInfo->schedNode));
            break;
        default:
            break;
        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdMoveRequetToReadyList - move request to ready list
*
* This routine moves request from sending list to ready list
*
* RETURNS: OK, or ERROR if move failed
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdMoveRequetToReadyList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,    /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo /* Pointer to the request */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return  ERROR;
        }

    /*
     * Add request into Ready list
     * The Caller entry shold make sure the synchronization of the list
     */

    switch (pHCDPipe->uEndpointType)
    {
    case USB_ATTR_CONTROL:
    case USB_ATTR_BULK:
        if (ERROR != lstFind(&(pHCDData->nonPeriodicReqList),
                        &(pRequestInfo->schedNode)))
        {
        lstDelete(&(pHCDData->nonPeriodicReqList),
                  &(pRequestInfo->schedNode));
        lstAdd(&(pHCDData->nonPeriodicReqReadyList),
               &(pRequestInfo->schedNode));
        return OK;
        }
        break;
    case USB_ATTR_INTERRUPT:
    case USB_ATTR_ISOCH:
        if (ERROR != lstFind(&(pHCDData->periodicReqList),
                        &(pRequestInfo->schedNode)))
        {
        lstDelete(&(pHCDData->periodicReqList),
                  &(pRequestInfo->schedNode));
        lstAdd(&(pHCDData->periodicReqReadyList),
               &(pRequestInfo->schedNode));
        return OK;
        }
        break;
    default:
        break;
    }
    return ERROR;
    }

/*******************************************************************************
*
* usbSynopsysHcdMoveRequetToSendList - move request to sending list
*
* This rountine moves the request from ready to sending list.
*
* RETURNS: OK, or ERROR if move failed
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdMoveRequetToSendList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,    /* Pointer to the HCDPipe */
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo /* Pointer to the request */
    )
    {
    if ((NULL == pHCDData) || (NULL == pHCDPipe) || (NULL == pRequestInfo))
        {
        USB_SHCD_ERR("Parameter not valid\n", 0, 0, 0, 0, 0, 0);
        return  ERROR;
        }
    /*
     * Move quest from send list into ready list
     * The Caller entry shold make sure the synchronization of the list
     */

    switch (pHCDPipe->uEndpointType)
        {
        case USB_ATTR_CONTROL:
        case USB_ATTR_BULK:
            if (ERROR != lstFind(&(pHCDData->nonPeriodicReqReadyList),
                                &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->nonPeriodicReqReadyList),
                          &(pRequestInfo->schedNode));
                /*
                 * Add the request to the tail of the list
                 * usbSynopsysHcdMoveRequetToSendList was called for re-schedule
                 * (NAK/NYET) if we add the reqeust to the head of the list
                 * the NAKed(NYETed) request will always be scheduled at first
                 * so other requests could not get chance to be scheduled
                 */

                lstAdd(&(pHCDData->nonPeriodicReqList),
                       &(pRequestInfo->schedNode));
                return OK;
                }
            break;
        case USB_ATTR_INTERRUPT:
        case USB_ATTR_ISOCH:
            if (ERROR != lstFind(&(pHCDData->periodicReqReadyList),
                                &(pRequestInfo->schedNode)))
                {
                lstDelete(&(pHCDData->periodicReqReadyList),
                          &(pRequestInfo->schedNode));
                /*
                 * Add the request to the head of the sending list
                 * Because the request will be rescheduled in next SOF interrupt
                 */

                lstInsert (&(pHCDData->periodicReqList),
                           NULL,
                           &(pRequestInfo->schedNode));
                return OK;
                }
            break;
        default:
            break;
        }
    return ERROR;
    }

/*******************************************************************************
*
* usbSynopsysHcdReqComplete - complete a URB 
*
* This routine is to complete a urb.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdUrbComplete
    (
    pUSBHST_URB            pUrb,
    int                    status
    )
    {
    if (pUrb != NULL)
        {
        if (pUrb->pHcdSpecific != NULL)
            {
            pUrb->pHcdSpecific = NULL;        
            pUrb->nStatus = (USBHST_STATUS)status;
            if (pUrb->pfCallback)
                (pUrb->pfCallback)(pUrb);
            }
        }
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdCleanPipe - release all the requests on the pipe 
*
* This routine release all the requests on the pipe. 
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdCleanPipe
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    /* Pointer to HCD block */
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe     /* Pointer to the HCDPipe */
    )
    {
    NODE *                        pNode = NULL;
    NODE *                        pNextNode = NULL;
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequest = NULL;

    if ((pHCDData == NULL) || (pHCDPipe == NULL))
        {
        USB_SHCD_ERR("Invalid Parameter\n", 1, 2, 3, 4, 5, 6);
        return;
        }
    
    pNode = lstFirst(&pHCDPipe->reqList);
    while (pNode != NULL)
        {
        pNextNode = lstNext(pNode);
        pRequest = LIST_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
        if (pRequest != NULL)
            {         
            usbSynopysHcdDeleteRequestFromScheduleList(pHCDData,
                                                       pHCDPipe,
                                                       pRequest);

            usbSynopysHcdDeleteRequestFromReadyList(pHCDData,
                                                    pHCDPipe,
                                                    pRequest);

            usbSynopsysHcdDeleteReq(pHCDPipe, pRequest);
            }
        pNode = pNextNode;
        }
    
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdDisableChannel - disable the assigned channel
*
* This routine disables the assigned channel.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL void usbSynopsysHcdDisableChannel
    (
    pUSB_SYNOPSYSHCD_DATA  pHCDData,
    int                    uChannel
    )
    {
    UINT32 uUsbHccharReg;
    
    /* Caller should verify the parameters's validation */
    
    uUsbHccharReg = 
        USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                   USB_SYNOPSYSHCD_HCCHAR(uChannel));
    
    /* Set CHENA and CHDIS both to "1" to disable the channel */

    uUsbHccharReg |= USB_SYNOPSYSHCD_HCCHAR_CHENA;
    uUsbHccharReg |= USB_SYNOPSYSHCD_HCCHAR_CHDIS;

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_HCCHAR(uChannel),
                                uUsbHccharReg);
    }

/*******************************************************************************
*
* usbSynopsysHcdHaltChannel - halt channel that may effect the transfer 
*
* This routine halts the channel that may effect the transfer.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdHaltChannel
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,   
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe    
    )
    {
    UINT8                 uChannel;
    UINT8                 uEpType;
    UINT32                uDmaChannel;
    UINT32                uUsbHccharReg = 0;
    UINT32                umask = 0;
    pUSB_SYNOPSYSHCD_PIPE pHaltHCDPipe = NULL;

    uDmaChannel = (~pHCDData->uIdleDmaChannelMap) & 
                  (~(1 << pHCDPipe->uDmaChannel));

    umask = (USB_SYNOPSYSHCD_HCCHAR_CHENA | 
             USB_SYNOPSYSHCD_HCCHAR_EPDIR);
        
    while (uDmaChannel != 0) 
        {
        uChannel = (UINT8)USB_SYNOPSYSHCD_GET_FIRST_IDLE_DMACHANNEL(uDmaChannel);
        
        uUsbHccharReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, 
                            USB_SYNOPSYSHCD_HCCHAR(uChannel));
        uEpType =  (uUsbHccharReg & USB_SYNOPSYSHCD_HCCHAR_EPTYPE) >>
                   USB_SYNOPSYSHCD_HCCHAR_EPTYPE_OFFSET;
        
        /* 
         * Check if the transfer is non-periodic in type and 
         * still in transfer
         */
        
        if (((uUsbHccharReg & umask) == umask) && 
            ((uEpType == USB_ATTR_CONTROL) || (uEpType == USB_ATTR_BULK)))
                {
                if ((pHaltHCDPipe = pHCDData->pPipeInChannel[uChannel]) != NULL)
                    {  
                    pHaltHCDPipe->uHalted++;
                    if (pHaltHCDPipe->uHalted > USB_SYNOPSYSHCD_PIPE_HALT_VALUE)
                        {
                        pHCDPipe->uHaltChannel |= (0x01 << uChannel);
                        usbSynopsysHcdDisableChannel(pHCDData, uChannel);
                        }
                    }
                }            
            uDmaChannel &= (~(1 << uChannel));
            }
        }

/*******************************************************************************
*
* usbSynopsysHcdUnHaltChannel - unhalt the channel for this transfer 
*
* This routine unhalts the channel for this transfer.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdUnHaltChannel
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,   
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe    
    )
    {
    UINT8                 uChannel;
    pUSB_SYNOPSYSHCD_PIPE pHaltHCDPipe;
    
     /* 
      * Check if any chanel is halted for this pipe
      * if yes, resume the transfer by enable the schedule 
      */
      
     while (pHCDPipe->uHaltChannel != 0)
         {
         uChannel = (UINT8)USB_SYNOPSYSHCD_FIND_FIRST_SET_BIT(pHCDPipe->uHaltChannel);
         
         if ((pHaltHCDPipe = pHCDData->pPipeInChannel[uChannel]) != NULL)
             pHaltHCDPipe->uHalted = 0;         
    
         pHCDPipe->uHaltChannel &= ~(0x01 << uChannel);
         }
     
     return;
    }

/*******************************************************************************
*
* usbSynopsysHcdCreateReq - create the USB_SYNOPSYSHCD_REQUEST_INFO structure 
*
* This routine creates the USB_SYNOPSYSHCD_REQUEST_INFO structure.
*
* RETURNS: pointer of the USB_SYNOPSYSHCD_REQUEST_INFO structure 
*
* ERRNO: N/A
*
* \NOMANUAL
*/

pUSB_SYNOPSYSHCD_REQUEST_INFO usbSynopsysHcdCreateReq
    (
    pUSB_SYNOPSYSHCD_PIPE pHCDPipe
    )
    {
    pUSB_SYNOPSYSHCD_REQUEST_INFO  pRequest = NULL;

    if (pHCDPipe == NULL)
        {
        USB_SHCD_ERR("pHCDPipe is NULL\n", 1, 2, 3, 4, 5, 6);
        return NULL;
        }
    
    pRequest = OSS_CALLOC(sizeof(USB_SYNOPSYSHCD_REQUEST_INFO));

    if (pRequest == NULL)
        {
        USB_SHCD_ERR("Cannot alloc memory for request\n", 1, 2, 3, 4, 5, 6);
        return NULL;
        }
    
    /* Exclusively access the list */

    if (OS_WAIT_FOR_EVENT(pHCDPipe->listEventID, WAIT_FOREVER) != OK)
        {
        USB_SHCD_ERR("Cannot get event ID\n", 1, 2, 3, 4, 5, 6);
        OSS_FREE(pRequest);
        return NULL;
        }

    lstAdd(&(pHCDPipe->reqList), &(pRequest->listNode));  
    (void)OS_RELEASE_EVENT(pHCDPipe->listEventID);

    return pRequest;
    }

/*******************************************************************************
*
* usbSynopsysHcdDeleteReq - Delete the USB_SYNOPSYSHCD_REQUEST_INFO structure 
*
* This routine deletes the USB_SYNOPSYSHCD_REQUEST_INFO structure.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdDeleteReq
    (
    pUSB_SYNOPSYSHCD_PIPE          pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO  pRequest
    )
    {
    if ((pHCDPipe == NULL) ||(pRequest == NULL))
        {
        USB_SHCD_ERR("Invalid parameter \n", 1, 2, 3, 4, 5, 6);
        return;
        }
    
    /* Exclusively access the list */

    if (OS_WAIT_FOR_EVENT(pHCDPipe->listEventID, WAIT_FOREVER) != OK)
        {
        USB_SHCD_ERR("Cannot get event ID\n", 1, 2, 3, 4, 5, 6);
        return;
        }

    if (pRequest->pHCDPipe != pHCDPipe)
        {
        (void)OS_RELEASE_EVENT(pHCDPipe->listEventID);
        return;
        }

    /* Move request from request list to free list */
    
    if (ERROR != lstFind(&(pHCDPipe->reqList), &(pRequest->listNode)))                                                           
        lstDelete(&(pHCDPipe->reqList), &(pRequest->listNode));  

    OS_MEMSET(pRequest, 0, sizeof(USB_SYNOPSYSHCD_REQUEST_INFO));
    OSS_FREE(pRequest);
    pRequest = NULL;
    
    (void)OS_RELEASE_EVENT(pHCDPipe->listEventID);
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdFirstReqGet - get first request of the pipe
*
* This routine gets the first request of the pipe.
*
* RETURNS: pointer of the first request or NULL if failed
*
* ERRNO: N/A
*
* \NOMANUAL
*/

pUSB_SYNOPSYSHCD_REQUEST_INFO usbSynopsysHcdFirstReqGet
    (
    pUSB_SYNOPSYSHCD_PIPE          pHCDPipe
    )
    {
    NODE * pNode = NULL;

    if (pHCDPipe == NULL)
        return NULL;
    
    pNode = lstFirst(&pHCDPipe->reqList);
    
    if (pNode != NULL)
        return LIST_NODE_TO_USB_SYNOPSYSHCD_REQUEST_INFO(pNode);
    else
        return NULL;
    }
    
/*******************************************************************************
*
* usbSynopsysHcdNewPipe - create the USB_SYNOPSYSHCD_PIPE structure
*
* This routine creates the USB_SYNOPSYSHCD_PIPE structure.
*
* RETURNS: pointer of the pUSB_SYNOPSYSHCD_PIPE
*
* ERRNO: N/A
*
* \NOMANUAL
*/

pUSB_SYNOPSYSHCD_PIPE usbSynopsysHcdNewPipe(void)
    {
    pUSB_SYNOPSYSHCD_PIPE pHCDPipe = NULL;
    
    pHCDPipe = (pUSB_SYNOPSYSHCD_PIPE)OSS_CALLOC(sizeof(USB_SYNOPSYSHCD_PIPE));

    /* Check if memory allocation is successful */

    if (NULL == pHCDPipe)
        {
        USB_SHCD_ERR("Memory not allocated for SynopsysHCD pipe\n",
                     0, 0, 0, 0, 0, 0);
        return NULL;
        }

    pHCDPipe->listEventID = semMCreate(SEM_Q_PRIORITY     |
                                       SEM_INVERSION_SAFE |
                                       SEM_DELETE_SAFE);

    if (pHCDPipe->listEventID == NULL)
        {
        USB_SHCD_ERR("Create listEventID fail\n", 0, 0, 0, 0, 0, 0);
        OSS_FREE(pHCDPipe);
        pHCDPipe = NULL;
        return NULL;
        }
     
    pHCDPipe->uPipeFlag = USB_SYNOPSYSHCD_PIPE_FLAG_OPEN;
    lstInit(&pHCDPipe->reqList);
    
    return pHCDPipe;
    }

/*******************************************************************************
*
* usbSynopsysHcdDestroyPipe - delete the USB_SYNOPSYSHCD_PIPE structure 
*
* This routine deletes the USB_SYNOPSYSHCD_PIPE structure.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdDestroyPipe
    (
    pUSB_SYNOPSYSHCD_PIPE pHCDPipe
    )
    {
    if (pHCDPipe == NULL)
        return;

    if (pHCDPipe->listEventID != NULL)
        {
        if (OK ==OS_WAIT_FOR_EVENT(pHCDPipe->listEventID, WAIT_FOREVER))
            {
            OS_DESTROY_EVENT(pHCDPipe->listEventID);
            pHCDPipe->listEventID = NULL;

            OSS_FREE(pHCDPipe);
            pHCDPipe = NULL;
            }
        else
            {
            USB_SHCD_ERR("Take pHCDPipe->listEventID failed 0x%x. \n",
                         pHCDPipe, 2, 3, 4, 5, 6);
            }
        }
    else
        {
        USB_SHCD_ERR("pHCDPipe->listEventID is NULL 0x%x. \n",
                     pHCDPipe, 2, 3, 4, 5, 6);
        }

    return;
    }

/* End of file */
