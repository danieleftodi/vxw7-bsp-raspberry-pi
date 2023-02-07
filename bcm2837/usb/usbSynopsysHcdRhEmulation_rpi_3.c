/* usbSynopsysHcdRhEmulation_rpi_3.c - roothub emulation entries for Synopsys
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

This file contains functions which handle the requests to the root hub.

INCLUDE FILES: usb/usbOsal.h, usb/usbHst.h, usb/usbHcdInstr.h,
               usbSynopsysHcdDataStructures_rpi_3.h,
               usbSynopsysHcdInterfaces_rpi_3.h,
               usbSynopsysHcdHardwareAccess_rpi_3.h,
               usbSynopsysHcdRegisterInfo_rpi_3.h
               usbSynopsysHcdRhEmulation_rpi_3.h,
               usbSynopsysHcdUtil_rpi_3.h
*/

/* includes */

#include <usbOsal.h>
#include <usbHst.h>
#include <usbHcdInstr.h>
#include "usbSynopsysHcdDataStructures_rpi_3.h"
#include "usbSynopsysHcdInterfaces_rpi_3.h"
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"
#include "usbSynopsysHcdRegisterInfo_rpi_3.h"
#include "usbSynopsysHcdRhEmulation_rpi_3.h"
#include "usbSynopsysHcdUtil_rpi_3.h"

/* locals */

/* Device Descriptor value for the Root hub */

LOCAL UCHAR gSynopsysHcdRHDeviceDescriptor[] = {
                                 0x12,        /* bLength */
                                 0x01,        /* Device Descriptor type */
                                 0x00, 0x02,  /* bcdUSB - USB 2.0 */
                                 0x09,        /* Hub DeviceClass */
                                 0x00,        /* bDeviceSubClass */
                                 0x01,        /* bDeviceProtocol High Speed single TT support*/
                                 0x40,        /* Max packet size is 64 */
                                 0x00, 0x00,  /* idVendor */
                                 0x00, 0x00,  /* idProduct */
                                 0x00, 0x00,  /* bcdDevice */
                                 0x00,        /* iManufacturer */
                                 0x00,        /* iProduct */
                                 0x00,        /* iSerialNumber */
                                 0x01         /* 1 configuration */
                                };

/*
 * Descriptor structure returned on a request for
 * Configuration descriptor for the Root hub.
 * This includes the Configuration descriptor, interface descriptor and
 * the endpoint descriptor
 */

LOCAL UCHAR gSynopsysHcdRHConfigDescriptor[] = {
                                /* Configuration descriptor */
                                0x09,        /* bLength */
                                0x02,        /* configuration descriptor type */
                                0x19, 0x00,  /* wTotalLength */
                                0x01,        /* 1 interface */
                                0x01,        /* bConfigurationValue */
                                0x00,        /* iConfiguration */
                                0xE0,        /* bmAttributes */
                                0x00,        /* bMaxPower */

                                /* Interface Descriptor */
                                0x09,        /* bLength */
                                0x04,        /* Interface Descriptor type */
                                0x00,        /* bInterfaceNumber */
                                0x00,        /* bAlternateSetting */
                                0x01,        /* bNumEndpoints */
                                0x09,        /* bInterfaceClass */
                                0x00,        /* bInterfaceSubClass */
                                0x00,        /* bInterfaceProtocol */
                                0x00,        /* iInterface */

                                /* Endpoint Descriptor */
                                0x07,        /* bLength */
                                0x05,        /* Endpoint Descriptor type */
                                0x81,         /* bEndpointAddress */
                                0x03,         /* bmAttributes */
                                0x08,   0x00, /* wMaxPacketSize */
                                0x0C          /* bInterval */
                              };
/* globals */

IMPORT pUSB_SYNOPSYSHCD_DATA *g_pSynopsysHCDData;
IMPORT BOOLEAN usbSynopsysHcdCopyRHInterruptData(
                                                 pUSB_SYNOPSYSHCD_DATA pHCDData,
                                                 UINT32      uStatusChange
                                                 );
/*
 * This is actually a dynamically allocated array
 * of size USB_MAX_SYNOPSYSHCI_COUNT.
 */

IMPORT spinlockIsr_t spinLockIsrSynopsysHcd[];

/* forward declarations */

USBHST_STATUS
usbSynopsysHcdRhProcessControlRequest(pUSB_SYNOPSYSHCD_DATA pHCDData,
                                      pUSBHST_URB    pURB);
USBHST_STATUS
usbSynopsysHcdRhProcessInterruptRequest(pUSB_SYNOPSYSHCD_DATA pHCDData,
                                        pUSBHST_URB    pURB);
USBHST_STATUS
usbSynopsysHcdRhProcessStandardRequest(pUSB_SYNOPSYSHCD_DATA pHCDData,
                                       pUSBHST_URB    pURB);
USBHST_STATUS
usbSynopsysHcdRhProcessClassSpecificRequest(pUSB_SYNOPSYSHCD_DATA pHCDData,
                                            pUSBHST_URB    pURB);
USBHST_STATUS
usbSynopsysHcdRhClearPortFeature(pUSB_SYNOPSYSHCD_DATA pHCDData,
                                 pUSBHST_URB    pURB);

USBHST_STATUS
usbSynopsysHcdRhGetHubDescriptor(pUSB_SYNOPSYSHCD_DATA pHCDData,
                                 pUSBHST_URB    pURB);

USBHST_STATUS
usbSynopsysHcdRhGetPortStatus(pUSB_SYNOPSYSHCD_DATA pHCDData,
                              pUSBHST_URB    pURB);

USBHST_STATUS
usbSynopsysHcdRhSetPortFeature(pUSB_SYNOPSYSHCD_DATA pHCDData,
                              pUSBHST_URB    pURB);


/*******************************************************************************
*
* usbSynopsysHcdRhCreatePipe - create a pipe specific to an endpoint
*
* This routine creates a pipe specific to an endpoint.
*
* RETURNS: USBHST_STATUS of the creat result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhCreatePipe
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,            /* Ptr to HCD block */
    UINT8                 uDeviceAddress,      /* Device Address */
    UINT8                 uDeviceSpeed,        /* Device Speed */
    UCHAR *               pEndpointDescriptor,/* Ptr to EndPoint Descriptor */
    ULONG *               puPipeHandle        /* Ptr to pipe handle */
    )
    {
    /* To hold the status of the function call */

    USBHST_STATUS Status = USBHST_FAILURE;

    /* Pointer to the Endpoint Descriptor */

    pUSBHST_ENDPOINT_DESCRIPTOR pEndpointDesc = NULL;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (USB_SYNOPSYSHCD_MAX_DEVICE_ADDRESS < uDeviceAddress) ||
        (NULL == pEndpointDescriptor) ||
        (NULL == puPipeHandle) ||
        (USBHST_HIGH_SPEED != uDeviceSpeed))
        {
        USB_SHCD_ERR("parameters are not valid\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the endpoint descriptor */

    pEndpointDesc = (pUSBHST_ENDPOINT_DESCRIPTOR)pEndpointDescriptor;

    /* Switch based on the endpoint type */

    switch (pEndpointDesc->bmAttributes & USB_SYNOPSYSHCD_ENDPOINT_TYPE_MASK)
        {
        /* Control endpoint */

        case USBHST_CONTROL_TRANSFER:
            {
            /* If the endpoint number is not 0, it is an error */

            if (0 != (pEndpointDesc->bEndpointAddress &
                     USB_SYNOPSYSHCD_ENDPOINT_NUMBER_MASK))
                {
                return USBHST_INVALID_REQUEST;
                }
            else
                {
                /* Allocate memory for the control endpoint */

                pHCDData->RHData.pControlPipe = usbSynopsysHcdNewPipe();

                /* Check if memory allocation is successful */

                if (NULL == pHCDData->RHData.pControlPipe)
                    {
                    USB_SHCD_ERR("Memory not allocated for control pipe\n",
                                 0, 0, 0, 0, 0, 0);

                    return USBHST_INSUFFICIENT_MEMORY;
                    }

                /* Populate the fields of the control pipe - Start */

                /* Copy the endpoint address */

                pHCDData->RHData.pControlPipe->uEndpointAddress =
                                                pEndpointDesc->bEndpointAddress;

                /* Copy the address */

                pHCDData->RHData.pControlPipe->uDeviceAddress = uDeviceAddress;

                /* Update the speed */

                pHCDData->RHData.pControlPipe->uSpeed =
                                                     USB_SYNOPSYSHCD_HIGH_SPEED;

                /* Update the endpoint type */

                pHCDData->RHData.pControlPipe->uEndpointType =
                                                        USBHST_CONTROL_TRANSFER;

                /* Set Data Toggle */

                pHCDData->RHData.pControlPipe->uPidToggle = 0;

                /* Populate the fields of the control pipe - End */

                /* Update the pipe handle information */

                *(puPipeHandle) = (ULONG) pHCDData->RHData.pControlPipe;

                Status = USBHST_SUCCESS;
                }
            break;
            }
        /* Interrupt endpoint */

        case USBHST_INTERRUPT_TRANSFER:
            {
            /* Allocate memory for the interrupt endpoint */

            pHCDData->RHData.pInterruptPipe = usbSynopsysHcdNewPipe();

            /* Check if memory allocation is successful */

            if (NULL == pHCDData->RHData.pInterruptPipe)
                {
                USB_SHCD_ERR("Memory not allocated for interrupt pipe\n",
                             0, 0, 0, 0, 0, 0);

                return USBHST_INSUFFICIENT_MEMORY;
                }

            /* Populate the fields of the interrupt pipe - Start */

            /* Copy the endpoint address */

            pHCDData->RHData.pInterruptPipe->uEndpointAddress =
                                                pEndpointDesc->bEndpointAddress;

            /* Copy the address */

            pHCDData->RHData.pInterruptPipe->uDeviceAddress = uDeviceAddress;

            /* Update the speed */

            pHCDData->RHData.pInterruptPipe->uSpeed = USB_SYNOPSYSHCD_HIGH_SPEED;

            /* Update the endpoint type */

            pHCDData->RHData.pInterruptPipe->uEndpointType =
                                                    USBHST_INTERRUPT_TRANSFER;

            /* Set Data Toggle */

            pHCDData->RHData.pInterruptPipe->uPidToggle = 0;

            /* Populate the fields of the interrupt pipe - End */

            /* Update the pipe handle information */

            *(puPipeHandle) = (ULONG) pHCDData->RHData.pInterruptPipe;

            Status = USBHST_SUCCESS;

            break;
            }
        default:
            {
            Status = USBHST_INVALID_REQUEST;
            }
        }
    /* End of switch */

    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRHDeletePipe - delete a pipe specific to an endpoint
*
* This routine deletes a pipe specific to an endpoint.
*
* RETURNS: USBHST_STATUS of the delete result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRHDeletePipe
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block */
    ULONG                 uPipeHandle         /* Pipe Handle Identifier */
    )
    {
     /* To hold the pointer to the pipe */

    pUSB_SYNOPSYSHCD_PIPE  pHCDPipe = NULL;

    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (0 == uPipeHandle))
        {
        USB_SHCD_ERR("Parameters not valid\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the USB_SYNOPSYSHCD_PIPE pointer */

    pHCDPipe = (pUSB_SYNOPSYSHCD_PIPE)uPipeHandle;

    /* Check if it is a control pipe delete request */

    if (pHCDPipe == pHCDData->RHData.pControlPipe)
        {
        usbSynopsysHcdDestroyPipe(pHCDData->RHData.pControlPipe);
        pHCDData->RHData.pControlPipe = NULL;
        }
    /* Check if it is an interrupt pipe delete request */

    else if (pHCDPipe == pHCDData->RHData.pInterruptPipe)
        {
        /* There may need some special action with the interrupt pipe */

        usbSynopsysHcdDestroyPipe(pHCDData->RHData.pInterruptPipe);
        pHCDData->RHData.pInterruptPipe = NULL;
        }
    else
        {
        USB_SHCD_ERR("Invalid pipe handle\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }


    return USBHST_SUCCESS;

    }

/*******************************************************************************
*
* usbSynopsysHcdRHSubmitURB - submit a request to an endpoint
*
* This routine submits a request to an endpoint.
*
* RETURNS: USBHST_STATUS of the submit result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRHSubmitURB
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block */
    ULONG                 uPipeHandle,        /* Pipe Handle Identifier */
    pUSBHST_URB           pURB                /* Ptr to User Request Block */
    )
    {

    /* Status of the request */

    USBHST_STATUS Status = USBHST_FAILURE;


    /* Check if the parameters are valid */

    if ((NULL == pHCDData) ||
        (0 == uPipeHandle) ||
        (NULL == pURB))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Check if it is a control request */

    if (uPipeHandle == (ULONG) (pHCDData->RHData.pControlPipe) ||
        ((uPipeHandle == (ULONG) pHCDData->pDefaultPipe) &&
        (0 == pHCDData->RHData.uDeviceAddress)))
        {
        Status = usbSynopsysHcdRhProcessControlRequest(pHCDData, pURB);
        }

    /* Check if it is an interrupt request */

    else if (uPipeHandle == (ULONG) (pHCDData->RHData.pInterruptPipe))
        {
        Status = usbSynopsysHcdRhProcessInterruptRequest(pHCDData, pURB);
        }
    else
        {
        USB_SHCD_ERR("Invalid pipe handle\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }


    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRHCancelURB - cancel a request submitted for an endpoint
*
* This routine cancels a request submitted for an endpoint.
*
* RETURNS: USBHST_STATUS of the cancel result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRHCancelURB
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block */
    ULONG                 uPipeHandle,        /* Pipe Handle Identifier */
    pUSBHST_URB           pURB                /* Ptr to User Request Block */
    )
    {

    /* Pointer to the HCD maintained pipe */

    pUSB_SYNOPSYSHCD_PIPE pHCDPipe = NULL;

    /* To hold the request information */

    pUSB_SYNOPSYSHCD_REQUEST_INFO  pRequestInfo = NULL;

    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (0 == uPipeHandle) ||
        (NULL == pURB))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);

        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the pipe data structure */

    pHCDPipe = (pUSB_SYNOPSYSHCD_PIPE)uPipeHandle;

    /* Search for the URB's request element */
    
    pRequestInfo = (pUSB_SYNOPSYSHCD_REQUEST_INFO)pURB->pHcdSpecific;

    /* If the request is not found, return an error */

    if ((NULL == pRequestInfo) || (NULL == pRequestInfo->pUrb))
        {
        USB_SHCD_ERR("Request is not present\n",
                     0, 0, 0, 0, 0, 0);

        return USBHST_INVALID_REQUEST;
        }

    /* Assert if the HCDPipe pointers do not match */

    if (pRequestInfo->pHCDPipe != pHCDPipe)
        {
        USB_SHCD_ERR("the pHCDPipe doesn't match\n",
                     0, 0, 0, 0, 0, 0);

        return USBHST_INVALID_REQUEST;
        }
    
    usbSynopsysHcdUrbComplete(pRequestInfo->pUrb, USBHST_TRANSFER_CANCELLED);
    
    usbSynopsysHcdDeleteReq(pHCDPipe, pRequestInfo);

    /* Return a success status */

    return USBHST_SUCCESS;
    }

/*******************************************************************************
*
* usbSynopsysHcdRhProcessControlRequest - process a control transfer request
*
* This routine processes a control transfer request.
*
* RETURNS: USBHST_STATUS of the processing result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhProcessControlRequest
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,    /* Ptr to HCD block */
    pUSBHST_URB           pURB         /* Ptr to User Request Block */
    )
    {
    /* Status of the request */

    USBHST_STATUS Status = USBHST_FAILURE;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the setup packet from the URB */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /* Check if it is a standard request */

    if (USB_SYNOPSYSHCD_RH_STANDARD_REQUEST ==
                        (USB_SYNOPSYSHCD_RH_REQUEST_TYPE & pSetup->bmRequestType))
        {
        Status = usbSynopsysHcdRhProcessStandardRequest(pHCDData, pURB);
        }

    /* Check if it is a class specific request */

    else if (USB_SYNOPSYSHCD_RH_CLASS_SPECIFIC_REQUEST ==
                    (USB_SYNOPSYSHCD_RH_REQUEST_TYPE & pSetup->bmRequestType))
        {
        Status = usbSynopsysHcdRhProcessClassSpecificRequest(pHCDData, pURB);
        }
    else
        {
        USB_SHCD_ERR("Invalid request\n", 0, 0, 0, 0, 0, 0);
        Status = USBHST_INVALID_PARAMETER;
        }

    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRhProcessInterruptRequest - processes a interrupt transfer request
*
* This routine processes a interrupt transfer request.
*
* RETURNS:  USBHST_STATUS of the processing result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhProcessInterruptRequest
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,    /* Ptr to HCD block */
    pUSBHST_URB           pURB         /* Ptr to User Request Block */
    )
    {
    /* To hold the return status of the function */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* To hold the interrupt data */

    UINT32 uInterruptData = 0;

    /* Pointer to the request information */

    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequest = NULL;

    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferBuffer))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /*
     * Check if the Root hub is configured
     * to accept any interrupt transfer request.
     */

    if ((0 == pHCDData->RHData.uConfigValue) ||
        (NULL == pHCDData->RHData.pInterruptPipe) ||
        (USB_SYNOPSYSHCD_PIPE_FLAG_DELETE & pHCDData->RHData.pInterruptPipe->uPipeFlag))
        {
        USB_SHCD_ERR("Invalid request - device not configured\n",
                     0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_REQUEST;
        }

    /* Assert if the structure members are invalid */

    OS_ASSERT(NULL != pHCDData->RHData.pHubInterruptData);

    /* Exclusively access the request resource */

    (void)OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, OS_WAIT_INFINITE);

    /* Copy the existing interrupt data  */

    OS_MEMCPY(&uInterruptData,
              pHCDData->RHData.pHubInterruptData,
              pHCDData->RHData.uSizeInterruptData);

    /* If interrupt data is available, copy the data directly to URB buffer */

    if (0 != uInterruptData)
        {
        OS_MEMCPY(pURB->pTransferBuffer,
                  &uInterruptData,
                  pHCDData->RHData.uSizeInterruptData);

        /* Initialize the interrupt data */

        OS_MEMSET(pHCDData->RHData.pHubInterruptData,
                  0,
                  pHCDData->RHData.uSizeInterruptData);

        /* Release the exclusive access */

        (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);

        /* Update the length */

        pURB->uTransferLength = pHCDData->RHData.uSizeInterruptData;

        /* Update the status of URB */

        pURB->nStatus = Status;

        /*
         * If a callback function is registered, call the callback
         * function.
         */

        if (pURB->pfCallback)
            {
            pURB->pfCallback(pURB);
            }
        }
    else
        {

        /* Allocate memory for the request data structure */

        pRequest = usbSynopsysHcdCreateReq(pHCDData->RHData.pInterruptPipe);

        /* Check if memory allocation is successful */

        if (NULL == pRequest)
            {
            USB_SHCD_ERR("memory not allocated\n", 0, 0, 0, 0, 0, 0);
            (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
            return USBHST_INSUFFICIENT_MEMORY;
            }

        /* Copy the USB_EHCD_PIPE pointer */

        pRequest->pHCDPipe = pHCDData->RHData.pInterruptPipe;

        /* Store the URB pointer in the request data structure */

        pRequest->pUrb = pURB;
        pURB->pHcdSpecific = pRequest;

        /* Release the exclusive access */

        (void)OS_RELEASE_EVENT(pHCDData->RequestSynchEventID);
        }
    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRhProcessStandardRequest - process a standard transfer request
*
* This routine processes a standard transfer request.
*
* RETURNS: USBHST_STATUS of the processing result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhProcessStandardRequest
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,    /* Ptr to HCD block */
    pUSBHST_URB           pURB         /* Ptr to User Request Block */
    )
    {
    /* To hold the return status of the function */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the setup packet from the URB */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /* Swap the 16 bit values */

    pSetup->wValue = OS_UINT16_LE_TO_CPU(pSetup->wValue);
    pSetup->wIndex = OS_UINT16_LE_TO_CPU(pSetup->wIndex);
    pSetup->wLength = OS_UINT16_LE_TO_CPU(pSetup->wLength);

    /* Handle the standard request */

    switch (pSetup->bRequest)
        {
        /* Clear feature request */

        case USBHST_REQ_CLEAR_FEATURE:
            {
            /* Based on the recipient, handle the request */

            switch (pSetup->bmRequestType & USB_SYNOPSYSHCD_RH_RECIPIENT_MASK)
                {
                /* Device recipient */

                case USBHST_RECIPIENT_DEVICE:
                    {
                    /* Check the feature selector */

                    if (USBHST_FEATURE_DEVICE_REMOTE_WAKEUP == pSetup->wValue)
                        {
                        /* Disable the device remote wakeup feature */

                        pHCDData->RHData.bRemoteWakeupEnabled = FALSE;
                        }
                        /* Update the URB status */

                        pURB->nStatus = USBHST_SUCCESS;
                        break;
                    }
                case USBHST_RECIPIENT_INTERFACE: /* Interface recipient */
                case USBHST_RECIPIENT_ENDPOINT:  /* Endpoint recipient */
                    {
                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;
                    break;
                    }
                default:
                    {
                    /* Invalid recipient value */

                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }
                }
            break;
            }
        /* Get Configuration request */

        case USBHST_REQ_GET_CONFIGURATION:
            {

            /*
             * Update the URB transfer buffer with the current configuration
             * value for the root hub
             */

            pURB->pTransferBuffer[0] = pHCDData->RHData.uConfigValue;

            /* Update the URB transfer length */

            pURB->uTransferLength = USB_SYNOPSYSHCD_RH_GET_CONFIG_SIZE;

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
        /* Get Descriptor request */

        case USBHST_REQ_GET_DESCRIPTOR:
            {
            /* Check the descriptor type */

            switch (pSetup->wValue >> USB_SYNOPSYSHCD_RH_DESCRIPTOR_BITPOSITION)
                {
                case USBHST_DEVICE_DESC:
                    {
                    /* Check the length of descriptor requested */

                    if (pSetup->wLength >= USB_SYNOPSYSHCD_RH_DEVICE_DESC_SIZE)
                        {
                        /* Update the URB transfer length */

                        pURB->uTransferLength = USB_SYNOPSYSHCD_RH_DEVICE_DESC_SIZE;
                        }
                    else
                        {
                        /* Update the URB transfer length */

                        pURB->uTransferLength = pSetup->wLength;
                        }

                    /* Copy the descriptor to the URB transfer buffer */

                    OS_MEMCPY(pURB->pTransferBuffer,
                              gSynopsysHcdRHDeviceDescriptor,
                              pURB->uTransferLength);


                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;

                    break;
                    }
                case USBHST_CONFIG_DESC:
                    {

                    /* Check the length of descriptor requested */

                    if (pSetup->wLength >= USB_SYNOPSYSHCD_RH_CONFIG_DESC_SIZE)
                        {
                        /* Update the URB transfer length */

                        pURB->uTransferLength = USB_SYNOPSYSHCD_RH_CONFIG_DESC_SIZE;
                        }
                    else
                        {
                        /* Update the URB transfer length */

                        pURB->uTransferLength = pSetup->wLength;
                        }

                    /* Copy the descriptor to the URB transfer buffer */

                    OS_MEMCPY(pURB->pTransferBuffer,
                              gSynopsysHcdRHConfigDescriptor,
                              pURB->uTransferLength);

                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;
                    break;
                    }
                default:
                    {
                    /* Invalid descriptor type */

                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }
                }
            break;
            }
        /* Get Status request */

        case USBHST_REQ_GET_STATUS:
            {
            /* Based on the recipient value, handle the request */

            switch (pSetup->bmRequestType & USB_SYNOPSYSHCD_RH_RECIPIENT_MASK)
                {
                /* Device recipient */

                case USBHST_RECIPIENT_DEVICE:
                    {
                    /* Clear the URB transfer buffer */

                    OS_MEMSET(pURB->pTransferBuffer,
                              0,
                              USB_SYNOPSYSHCD_RH_GET_STATUS_SIZE);

                    /* Update the device status - Self powered */

                    pURB->pTransferBuffer[0] = 0x01;

                    /* If remote wakeup is enabled, update the status */

                    if (TRUE == pHCDData->RHData.bRemoteWakeupEnabled)
                        {
                        /* Remote wakeup is enabled */

                        pURB->pTransferBuffer[0] |= 0x02;
                        }

                    /* Update the URB transfer length */

                    pURB->uTransferLength = USB_SYNOPSYSHCD_RH_GET_STATUS_SIZE;

                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;

                    break;
                    }
                case USBHST_RECIPIENT_INTERFACE: /* Interface recipient */
                case USBHST_RECIPIENT_ENDPOINT:  /* Endpoint recipient */
                    {
                    /* Update the URB transfer buffer */

                    OS_MEMSET(pURB->pTransferBuffer,
                              0,
                              USB_SYNOPSYSHCD_RH_GET_STATUS_SIZE);

                    /* Update the URB transfer length */

                    pURB->uTransferLength = USB_SYNOPSYSHCD_RH_GET_STATUS_SIZE;

                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;

                    break;
                    }
                default :
                    {
                    /* Invalid recipient value */

                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }

                }
            break;
            }
        /* Set Address request */

        case USBHST_REQ_SET_ADDRESS:
            {
            /* Check whether the address is valid */

            if (0 == pSetup->wValue)
                {
                /* Address is not valid */

                pURB->nStatus = USBHST_INVALID_REQUEST;

                break;
                }

            /* Update the RH address */

            pHCDData->RHData.uDeviceAddress = (UINT8)pSetup->wValue;

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;

            break;
            }
        /* Set Configuration request */

        case USBHST_REQ_SET_CONFIGURATION:
            {
            /* Check whether the configuration value is valid */

            if ((0 != pSetup->wValue) && (1 != pSetup->wValue))
                {
                /* Invalid configuration value. Update the URB status */

                pURB->nStatus = USBHST_INVALID_REQUEST;

                break;
                }

            /* Update the current configuration value for the root hub */

            pHCDData->RHData.uConfigValue = (UINT8)pSetup->wValue;

            /* Now the root hub is configured we can enable the root hub interrupt */

            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_GINTMSK,
                                        (USB_SYNOPSYSHCD_GINTSTS_INTERRUPT_MASK));

            USB_SHCD_DBG("enable the interrupt \n", 1, 2, 3, 4, 5, 6);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;

            break;
            }
        /* Set feature request */

        case USBHST_REQ_SET_FEATURE:
            {
            /* Based on the recipient, handle the request */

            switch (pSetup->bmRequestType & USB_SYNOPSYSHCD_RH_RECIPIENT_MASK)
                {
                /* Device recipient */

                case USBHST_RECIPIENT_DEVICE:
                    {
                    /* Check the feature selector */

                    if (USBHST_FEATURE_DEVICE_REMOTE_WAKEUP == pSetup->wValue)
                        {
                        /* Disable the device remote wakeup feature */

                        pHCDData->RHData.bRemoteWakeupEnabled = TRUE;
                        }

                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;
                    break;
                    }
                case USBHST_RECIPIENT_INTERFACE: /* Interface recipient */
                case USBHST_RECIPIENT_ENDPOINT:  /* Endpoint recipient */
                    {
                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;
                    break;
                    }
                default:
                    {
                    /* Invalid recipient value */

                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }
                }
            break;
            }
        /* Invalid request */
        default :
            {
            pURB->nStatus = USBHST_INVALID_REQUEST;
            break;
            }
        }

    /* If a callback function is registered, call the callback function */

    if (NULL != pURB->pfCallback)
        {
        (pURB->pfCallback)(pURB);
        }

    return Status;

    }

/*******************************************************************************
*
* usbSynopsysHcdRhProcessClassSpecificRequest - process a class specific request
*
* This routine processes a class specific request.
*
* RETURNS: USBHST_STATUS of the processing result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhProcessClassSpecificRequest
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,    /* Ptr to HCD block */
    pUSBHST_URB           pURB         /* Ptr to User Request Block */
    )
    {
    /* To hold the request status */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }
    /*
     * Check if the Root hub is configured
     * to accept any class specific request
     */

    if (0 == pHCDData->RHData.uConfigValue)
        {
        USB_SHCD_ERR("Invalid request - device not configured\n",
                     0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_REQUEST;
        }
    /* Extract the setup packet from the URB */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /* Swap the 16 bit values */

    pSetup->wValue = OS_UINT16_LE_TO_CPU(pSetup->wValue);
    pSetup->wIndex = OS_UINT16_LE_TO_CPU(pSetup->wIndex);
    pSetup->wLength = OS_UINT16_LE_TO_CPU(pSetup->wLength);

    /* Handle the class specific request */

    switch (pSetup->bRequest)
        {
        /* Clear feature request */

        case USBHST_REQ_CLEAR_FEATURE:
            {
            /* Based on the recipient value, handle the request */

            switch (pSetup->bmRequestType & USB_SYNOPSYSHCD_RH_RECIPIENT_MASK)
                {
                case USBHST_RECIPIENT_DEVICE:
                    {
                    /* None of the hub class features are supported */

                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }
                /* Clear Port Feature request */

                case USBHST_RECIPIENT_OTHER:
                    {
                    /*
                     * Call the function which handles
                     * the clear port feature request
                     */

                    Status = usbSynopsysHcdRhClearPortFeature(pHCDData, pURB);
                    break;
                    }
                default :
                    {
                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }

                }
            break;
            }
        /* Hub Get Descriptor request */

        case USBHST_REQ_GET_DESCRIPTOR:
            {
            /*
             * Call the function to handle the
             * Root hub Get descripor request.
             */

            Status = usbSynopsysHcdRhGetHubDescriptor(pHCDData, pURB);
            break;
            }
        /* Get status request */

        case USBHST_REQ_GET_STATUS:
            {
            /* Based on the recipient value, handle the request */

            switch (pSetup->bmRequestType & USB_SYNOPSYSHCD_RH_RECIPIENT_MASK)
                {
                case USBHST_RECIPIENT_DEVICE:
                    {
                    /*
                     * The hub status cannot be retrieved. So send the status
                     * always as zero
                     */

                    OS_MEMSET(pURB->pTransferBuffer,
                              0,
                              USB_SYNOPSYSHCD_HUBSTATUS_SIZE);

                    /* Update the URB status */

                    pURB->nStatus = USBHST_SUCCESS;
                    break;
                    }
                /* GetPortStatus request */

                case USBHST_RECIPIENT_OTHER:
                    {
                    /*
                     * Call the function which handles
                     * the Get Port Status request
                     */

                    Status = usbSynopsysHcdRhGetPortStatus(pHCDData, pURB);
                    break;
                    }
                default :
                    {
                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }
                }
            break;
            }
        /* Set Feature request */

        case USBHST_REQ_SET_FEATURE:
            {
            /* Based on the recipient value, handle the request */

            switch (pSetup->bmRequestType & USB_SYNOPSYSHCD_RH_RECIPIENT_MASK)
                {
                case USBHST_RECIPIENT_DEVICE:
                    {
                    /*
                     * None of the sethubfeature requests are supported
                     * in the Root hub
                     */
                    /* Update the URB status */

                    pURB->nStatus = USBHST_INVALID_REQUEST;

                    break;
                    }
                /* GetPortStatus request */

                case USBHST_RECIPIENT_OTHER:
                    {
                    /*
                     * Call the function which handles
                     * the Get Port Status request
                     */

                    Status = usbSynopsysHcdRhSetPortFeature(pHCDData, pURB);
                    break;
                    }
                /* Invalid request */

                default :
                    {
                    pURB->nStatus = USBHST_INVALID_REQUEST;
                    break;
                    }

                }

            break;
            }
        default:
            {
            /* Invalid request */

            USB_SHCD_WARN("usbSynopsysHcdRhProcessClassSpecificRequest - "
                          "USBHST_INVALID_REQUEST :\n"
                          "bRequest 0x%x wValue 0x%x wIndex 0x%x wLength 0x%x\n",
                          pSetup->bRequest,
                          pSetup->wValue,
                          pSetup->wIndex,
                          pSetup->wLength, 0, 0);

            pURB->nStatus = USBHST_INVALID_REQUEST;
            }

        }

    /* If a callback function is registered, call the callback function */

    if (NULL != pURB->pfCallback)
        {
        (pURB->pfCallback)(pURB);
        }


    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRhClearPortFeature - clear a feature of the port
*
* This routine clears a feature of the port.
*
* RETURNS:  USBHST_STATUS of the clear result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhClearPortFeature
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block */
    pUSBHST_URB           pURB                /* Ptr to User Request Block */
    )
    {
    /* To hold the request status */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;

    /* index of the host controller */

    UINT32 uBusIndex = 0;

    /* To hold the status of the hub port */

    UINT32 uPortStatus = 0;

    UINT32 uTempReg = 0;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);

        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the index of the host controller */

    uBusIndex = pHCDData->uBusIndex;

    /* Extract the setup packet */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /* Check whether the members are valid */

    if (pSetup->wIndex >
                pHCDData->RHData.uNumPorts)
        {
        USB_SHCD_ERR("Invalid port index\n", 0, 0, 0, 0, 0, 0);
        pURB->nStatus = USBHST_INVALID_PARAMETER;
        return USBHST_INVALID_PARAMETER;
        }

    /* Handle the request based on the feature to be cleared */

    switch (pSetup->wValue)
        {
        case USB_HUB_FSEL_PORT_ENABLE:/* Port enable */
            {
            /* R/W1C register, write 1 to clear the enable*/

            USB_SYNOPSYSHCD_SETBITS32_REG(pHCDData,
                                          USB_SYNOPSYSHCD_HPRT,
                                          USB_SYNOPSYSHCD_HPRT_PRTENA);

            /* Clear the related bits in uHPRTStatus */
            
            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            pHCDData->uHPRTStatus &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            /* Now we cann't get the changed frame number */

            pHCDData->uMicroFrameNumChangeFlag = 0;

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
        case USB_HUB_FSEL_PORT_SUSPEND:/* Port Suspend */
            {
            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT);
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
            uTempReg |= USB_SYNOPSYSHCD_HPRT_PRTRES;
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT, uTempReg);

            (void)OS_DELAY_MS(100);

            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT);
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTRES;
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT, uTempReg);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;

            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data to the LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Clear the suspend bit */

            uPortStatus &= (UINT32)~(USB_HUB_STS_PORT_SUSPEND);

            /* Update the suspend change */

            uPortStatus |= (USB_HUB_C_PORT_SUSPEND << 16);

            /* Swap the data to CPU format data */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);
            /*
             * Call the function to populate the interrupt
             * status data
             */

            (void) usbSynopsysHcdCopyRHInterruptData(pHCDData,
                     (UINT32)(USB_SYNOPSYSHCD_RH_MASK_VALUE << (pSetup->wIndex -1)));

            break;
            }
        case USB_HUB_FSEL_PORT_POWER:  /* Port Power */
            {
            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT);
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTPWR;
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT, uTempReg);

            /* Clear the related bits in uHPRTStatus */

            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            pHCDData->uHPRTStatus &= ~USB_SYNOPSYSHCD_HPRT_PRTPWR;
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
        case USB_HUB_FSEL_C_PORT_CONNECTION:/* c_port_connection */
            {
            /* R/W1C register, write 1 to clear the connection change */

            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_HPRT);
            uTempReg |= USB_SYNOPSYSHCD_HPRT_PRTCONNDET;
            uTempReg &= ~(USB_SYNOPSYSHCD_HPRT_PRTENA);

            USB_SYNOPSYSHCD_WRITE32_REG (pHCDData,
                                         USB_SYNOPSYSHCD_HPRT,
                                         uTempReg);

            /* Clear the related bits in uHPRTStatus */

            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            pHCDData->uHPRTStatus &= ~USB_SYNOPSYSHCD_HPRT_PRTCONNDET;
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);


            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus + 
                       ((pSetup->wIndex) -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data from the LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                        uPortStatus);

            /* Update the connect status change */

            uPortStatus &= (UINT32)~(USB_HUB_C_PORT_CONNECTION << 16);

            /* Swap the data to LE format data */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                        uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus + 
                       ((pSetup->wIndex) -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
        case USB_HUB_FSEL_C_PORT_ENABLE:/* c_port_enabled */
            {
            /* R/W1C register, write 1 to clear the enable change */

            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_HPRT);
            uTempReg |= USB_SYNOPSYSHCD_HPRT_PRTENCHNG;
            uTempReg &= ~(USB_SYNOPSYSHCD_HPRT_PRTENA);

            USB_SYNOPSYSHCD_WRITE32_REG (pHCDData,
                                         USB_SYNOPSYSHCD_HPRT,
                                         uTempReg);

            /* Clear the related bits in uHPRTStatus */

            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            pHCDData->uHPRTStatus &= ~USB_SYNOPSYSHCD_HPRT_PRTENCHNG;
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            /* Clear the internal data for port enable change */
            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus + 
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data to the LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Update the reset change */

            uPortStatus &= (UINT32)~(USB_HUB_C_PORT_ENABLE << 16);

            /* Swap the data to CPU format data */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
        case USB_HUB_FSEL_C_PORT_OVER_CURRENT:/* c_port_over_current */
            {
            /* R/W1C register, write 1 to clear the OverCurrent change */

            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_HPRT);
            uTempReg |= USB_SYNOPSYSHCD_HPRT_PRTOVRCURRCHNG;
            uTempReg &= ~(USB_SYNOPSYSHCD_HPRT_PRTENA);
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HPRT,
                                        uTempReg);

            /* Clear the related bits in uHPRTStatus */

            SPIN_LOCK_ISR_TAKE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);
            pHCDData->uHPRTStatus &= ~USB_SYNOPSYSHCD_HPRT_PRTOVRCURRCHNG;
            SPIN_LOCK_ISR_GIVE(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex]);

            /* Clear the internal data for port over current change */
            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus + 
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data to the LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Update the reset change */

            uPortStatus &= (UINT32)~(USB_HUB_C_PORT_OVER_CURRENT << 16);

            /* Swap the data to CPU format data */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
        case USB_HUB_FSEL_C_PORT_RESET:/* c_port_reset */
            {
            /* Clear the internal data for reset change */
            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus + 
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data to the LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Update the reset change */

            uPortStatus &= (UINT32)~(USB_HUB_C_PORT_RESET << 16);

            /* Swap the data to CPU format data */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }

        case USB_HUB_FSEL_C_PORT_SUSPEND:/* c_port_suspend */
            {
            /* Clear the internal data for port suspend change */
            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus + 
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data to the LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Update the reset change */

            uPortStatus &= (UINT32)~(USB_HUB_C_PORT_SUSPEND << 16);

            /* Swap the data to CPU format data */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;

            /*
             * Call the function to populate the interrupt
             * status data
             */

            (void) usbSynopsysHcdCopyRHInterruptData(pHCDData,
                    (UINT32)(USB_SYNOPSYSHCD_RH_MASK_VALUE << (pSetup->wIndex -1)));
            break;
            }
        default:  /* Unknown feature */
            {
            pURB->nStatus = USBHST_INVALID_REQUEST;
            break;
            }/* End of default */
        }/* End of switch () */

    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRhGetHubDescriptor - get the hub descriptor
*
* This routine gets the hub descriptor.
*
* RETURNS:  USBHST_STATUS of the get result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhGetHubDescriptor
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block */
    pUSBHST_URB           pURB                /* Ptr to User Request Block */
    )
    {
    /* To hold the request status */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;

    /* To hold the number of bytes alloted for the port information */

    UINT32 uPortBytes = 0;

    /* To hold the index into the ports */

    UINT32 uIndex = 0;

    /* Pointer to the buffer */

    UCHAR * pBuffer = NULL;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData) ||
        (NULL == pURB->pTransferBuffer))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);

        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the setup packet */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /*
     * Check whether there are any invalid conditions.
     * i.e invalid DevRequest parameters
     */

    if (0x2900 != pSetup->wValue ||
        0 != pSetup->wIndex)
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        pURB->nStatus = USBHST_INVALID_PARAMETER;
        return USBHST_INVALID_PARAMETER;
        }

    /*
     * Determine the number of bytes that the descriptor would occupy
     * ie. the num of bytes req to accomodate info about the ports
     */

    uPortBytes = (pHCDData->RHData.uNumPorts) / 8;
    if (0 != pHCDData->RHData.uNumPorts % 8)
        {
        uPortBytes++;
        }

    /* Allocate memory for the buffer */

    pBuffer = (UCHAR *)OS_MALLOC(7 + (uPortBytes * 2));

    /* Check if memory allocation is successful */

    if (NULL == pBuffer)
        {
        USB_SHCD_ERR("Memory not allocated\n", 0, 0, 0, 0, 0, 0);
        pURB->nStatus = USBHST_MEMORY_NOT_ALLOCATED;
        return USBHST_MEMORY_NOT_ALLOCATED;
        }

    /* Population of the values of hub descriptor - Start */

    /* Length of the descriptor */

    pBuffer[0] = (UINT8)(7 + (uPortBytes * 2));

    /* Hub Descriptor type */

    pBuffer[1] = 0x29;

    /* Number of downstream ports */

    pBuffer[2] = pHCDData->RHData.uNumPorts;

    /*
     * The following 2 bytes give the hub characteristics
     * The root hub has individual port overcurrent indication
     */

    pBuffer[3] = 0x08;
    pBuffer[4] = 0;

    /* The power On to Power Good Time for the Root hub is 1 * 2ms */

    pBuffer[5] = 1;

    /* There are no specific maximim current requirements */

    pBuffer[6] = 0;

    /* The last few bytes of the descriptor is based on the number of ports */

    for (uIndex = 0;uIndex < uPortBytes ; uIndex++)
        {
        /* Indicates whether the hub is removable */

        pBuffer[7+uIndex] = 0;

        /* Port power control mask should be 1 for all the ports */

        pBuffer[7+uPortBytes+uIndex] = 0xff;
        }

    /* Population of the values of hub descriptor - End */

    /* Update the length */

    if (pURB->uTransferLength >= 7 + (uPortBytes * 2))
        {
        pURB->uTransferLength = 7 + (uPortBytes * 2);
        }

    /* Copy the data */

    OS_MEMCPY(pURB->pTransferBuffer, pBuffer, pURB->uTransferLength);

    /* Update the status */

    pURB->nStatus = USBHST_SUCCESS;

    /* Free memory allocated for the buffer */

    OS_FREE(pBuffer);

    return Status;
    }
/* End of usbSynopsysHcdRhGetHubDescriptor() */

/*******************************************************************************
*
* usbSynopsysHcdRhGetPortStatus - get the status of the port
*
* This routine gets the status of the port.
*
* RETURNS:  USBHST_STATUS of the get result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhGetPortStatus
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block           */
    pUSBHST_URB           pURB                /* Ptr to User Request Block  */
    )
    {
    /* To hold the request status */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;

    /* To hold the status of the hub port */

    UINT32 uPortStatus = 0;

    /* To hold the status of the interrupt status */

    UINT32 uInterruptStatus = 0;

    /*
     * This holds the port status which is available in the
     * root hub port status buffer
     */

    UINT32 uPortStatusAvailable = 0;

    /* Index of the host controller */

    UINT32 uBusIndex = 0;

    /* To hold the register value */

    volatile UINT32 uTempReg = 0;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData) ||
        (NULL == pURB->pTransferBuffer))
        {
         USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the index of the host controller */

    uBusIndex = pHCDData->uBusIndex;

    /* Extract the setup packet */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /* Check whether the members are valid */

    if (pSetup->wIndex >
                pHCDData->RHData.uNumPorts)
        {
        USB_SHCD_ERR("Invalid port index\n", 0, 0, 0, 0, 0, 0);
        pURB->nStatus = USBHST_INVALID_PARAMETER;
        return USBHST_INVALID_PARAMETER;
        }

    /*
     * The hub's port status contains 4 bytes of information, out of which
     * the hub's port status is reported in the byte offsets 0 to 1
     * and the hub's port status change in the byte offsets 2 to 3
     */

    /* Update the port status change bytes - Start */

    /*
     * Following is the interpretation of the 16 bits of port status change
     * Bit 13-15 Reserved
     * Bit 12    Port Indicate Control
     * Bit 11    Port Test Mode
     * Bit 10    High Speed Device Attach
     * Bit 9     Low Speed Device Attach
     * Bit 8     Port Power
     * Bit 5-7   Reserved
     * Bit 4     Reset change
     * Bit 3     Overcurrent indicator Change
     * Bit 2     Suspend Change
     * Bit 1     Port Enable/Disable change
     * Bit 0     Connect status change
     */

    /* Copy the value in the port status register */

    OS_MEMCPY(&uPortStatusAvailable,
              (pHCDData->RHData.pPortStatus + 
               ((pSetup->wIndex) -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
              USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

    /* Swap the data from the LE format */

    uPortStatusAvailable = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                         uPortStatusAvailable);
    uPortStatus |= uPortStatusAvailable;

    /*
     * if chip in device mode or prepare to be,
     * the HPRT register cann't be access
     * so the judgement should avoid this possibility
     */

    uInterruptStatus = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_GINTSTS);

    /* if current mode is host mode, we will do the following */

    if (USB_SYNOPSYSHCD_DEV_MODE !=
      (uInterruptStatus & USB_SYNOPSYSHCD_GINTSTS_CURMOD))
        {
        uTempReg = USB_SYNOPSYSHCD_READ32_REG (pHCDData, USB_SYNOPSYSHCD_HPRT);
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTCONNSTS))
            {
            uPortStatus |= USB_HUB_STS_PORT_CONNECTION;
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTCONNDET))
            {
            uPortStatus |= (USB_HUB_C_PORT_CONNECTION << 16);
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTENA))
            {
            uPortStatus |= USB_HUB_STS_PORT_ENABLE;
            }
        else
            {
            uPortStatus &= (UINT32)~USB_HUB_STS_PORT_ENABLE;
            }
            

       /*
        * The Enable change means the port from enable to disable
        * So, if the change happens we can open report the status
        * when port is disable or else the hub will remove the device
        */

        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTENCHNG))
            {
            if ((uTempReg & USB_SYNOPSYSHCD_HPRT_PRTENA)== 0x0)
	            {
        	    uPortStatus |= (USB_HUB_C_PORT_ENABLE << 16);
       		    }
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTOVRCURRACT))
            {
            uPortStatus |= USB_HUB_STS_PORT_OVER_CURRENT;
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTOVRCURRCHNG))
            {
            uPortStatus |= (USB_HUB_C_PORT_OVER_CURRENT << 16);
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTSUSP))
            {
            uPortStatus |= USB_HUB_STS_PORT_SUSPEND;
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTRST))
            {
            uPortStatus |= USB_HUB_STS_PORT_RESET;
            }
        if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTPWR))
            {
            uPortStatus |= USB_HUB_STS_PORT_POWER;
            }
        if (USB_SYNOPSYSHCD_HPRT_PRTSPD_HIGH ==
          (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTSPD))
            {
            uPortStatus |= USB_HUB_STS_PORT_HIGH_SPEED;
            }
        if (USB_SYNOPSYSHCD_HPRT_PRTSPD_LOW ==
            (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTSPD))
            {
            uPortStatus |= USB_HUB_STS_PORT_LOW_SPEED;
            }

        }

    /* Swap the data to CPU format */

    uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex, uPortStatus);

    /* Copy the port status */

    OS_MEMCPY(pURB->pTransferBuffer, &uPortStatus, 4);

    /* Update the status of URB */

    pURB->nStatus = USBHST_SUCCESS;

    return Status;
    }

/*******************************************************************************
*
* usbSynopsysHcdRhSetPortFeature - set the features of the port
*
* This routine sets the features of the port.
*
* RETURNS:  USBHST_STATUS of the set result.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

USBHST_STATUS usbSynopsysHcdRhSetPortFeature
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,           /* Ptr to HCD block */
    pUSBHST_URB           pURB                /* Ptr to User Request Block */
    )
    {
    /* To hold the request status */

    USBHST_STATUS Status = USBHST_SUCCESS;

    /* Pointer to the setup packet */

    pUSBHST_SETUP_PACKET pSetup = NULL;

    /* Index of the host controller */

    UINT32 uBusIndex = 0;

    volatile UINT32 uTempReg = 0;

    UINT32 uPortStatus = 0;


    /* Check the validity of the parameters */

    if ((NULL == pHCDData) ||
        (NULL == pURB) ||
        (NULL == pURB->pTransferSpecificData))
        {
        USB_SHCD_ERR("Invalid parameters\n", 0, 0, 0, 0, 0, 0);
        return USBHST_INVALID_PARAMETER;
        }

    /* Extract the index of the host controller */

    uBusIndex = pHCDData->uBusIndex;

    /* Extract the setup packet */

    pSetup = (pUSBHST_SETUP_PACKET)pURB->pTransferSpecificData;

    /* If the device isn't connect we will return directly */

    /* Switch on the feature to be selected */

    switch (pSetup->wValue)
        {
        case USB_HUB_FSEL_PORT_RESET:/* If port reset */
            {
            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_HPRT);
            if (0 != (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTCONNSTS))
                {
                /*
                 * If the first command "get descriptor" doesn't success
                 * the statemachine will call here again and try to reset
                 * the port.So if the port is already enable, we still need
                 * to do the reset action.
                 */

                if (pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_ALTERA_SOC_GEN5
                    || pHCDData->uPlatformType == USB_SYNOPSYSHCI_PLATFORM_RPI_3)
                    {
                    /* Bit0-5 are read-only. Write 1 to bit2(port enable) will disable port. */

                    USB_SYNOPSYSHCD_SETBITS32_HPRT_REG(pHCDData,
                                                       USB_SYNOPSYSHCD_HPRT,
                                                       USB_SYNOPSYSHCD_HPRT_PRTRST);
                    (void)OS_DELAY_MS(60);
                    USB_SYNOPSYSHCD_CLEARBITS32_HPRT_REG(pHCDData,
                                                         USB_SYNOPSYSHCD_HPRT,
                                                         USB_SYNOPSYSHCD_HPRT_PRTRST);
                    }
                else
                    {
                    USB_SYNOPSYSHCD_SETBITS32_REG(pHCDData,
                                                  USB_SYNOPSYSHCD_HPRT,
                                                  USB_SYNOPSYSHCD_HPRT_PRTRST);
                    (void)OS_DELAY_MS(60);
                    USB_SYNOPSYSHCD_CLEARBITS32_REG(pHCDData,
                                                    USB_SYNOPSYSHCD_HPRT,
                                                    USB_SYNOPSYSHCD_HPRT_PRTRST);
                    }

                (void)OS_DELAY_MS(1000);
                uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData,
                                                      USB_SYNOPSYSHCD_HPRT);
                usbSynopsysHcdFlushRxFIFO(pHCDData);
                usbSynopsysHcdFlushTxFIFO(pHCDData,0x10);

                /* Check the port enable or not */

                if (0 == (uTempReg & USB_SYNOPSYSHCD_HPRT_PRTENA))
                    {
                    USB_SHCD_ERR("Unable to reset Port 0x%x.\n", uTempReg, 0, 0, 0, 0, 0);
                    pURB->nStatus = USBHST_FAILURE;
                    break;
                    }
                else
                    {
                    /* Copy the value in the port status register */

                    pHCDData->uMicroFrameNumChangeFlag = 1;

                    OS_MEMCPY(&uPortStatus,
                              (pHCDData->RHData.pPortStatus +
                               (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                              USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

                    /* Swap the data to LE format */

                    uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                                uPortStatus);

                    /* Update the reset change */

                    uPortStatus |= (USB_HUB_C_PORT_RESET << 16);

                    /* Swap the data to LE format */

                    uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                                uPortStatus);

                    /* Copy the status back to the port status */

                    OS_MEMCPY((pHCDData->RHData.pPortStatus +
                               (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                              &uPortStatus,
                              USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

                    /*
                     * Call the function to populate the interrupt
                     * status data
                     */

                    pURB->nStatus = USBHST_SUCCESS;

                    (void) usbSynopsysHcdCopyRHInterruptData(pHCDData,
                                      (UINT32)(USB_SYNOPSYSHCD_RH_MASK_VALUE <<
                                               (pSetup->wIndex -1)));
                    break;
                    }
                }
            else
                {
                pURB->nStatus = USBHST_SUCCESS;
                break;
                }
            }
        case USB_HUB_FSEL_PORT_SUSPEND: /* If port suspend */
            {
            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT);
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
            uTempReg |= USB_SYNOPSYSHCD_HPRT_PRTSUSP;
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT, uTempReg);

            /* Suspend the Phy clock */

            USB_SYNOPSYSHCD_SETBITS32_REG(pHCDData,
                                          USB_SYNOPSYSHCD_PCGCCTL,
                                          USB_SYNOPSYSHCD_PCGCCTL_STOPPCLK);

            /* if the chip have HNP cable, should delay at least 200 ms */
            /* The CN50XX doesn't have HNP calbe, so no need now */

            /* Update the status */

            pURB->nStatus = USBHST_SUCCESS;

            /* Copy the value in the port status register */

            OS_MEMCPY(&uPortStatus,
                      (pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);

            /* Swap the data to LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                        uPortStatus);

            /* Update the suspend change */

            uPortStatus |= ((USB_HUB_C_PORT_SUSPEND << 16) |
                            USB_HUB_STS_PORT_SUSPEND);

            /* Swap the data to LE format */

            uPortStatus = USB_SYNOPSYSHCD_SWAP_USB_DATA(uBusIndex,
                                                        uPortStatus);

            /* Copy the status back to the port status */

            OS_MEMCPY((pHCDData->RHData.pPortStatus +
                       (pSetup->wIndex -1) * USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE),
                      &uPortStatus,
                      USB_SYNOPSYSHCD_RH_PORT_STATUS_SIZE);
            /*
             * Call the function to populate the interrupt
             * status data
             */

            (void) usbSynopsysHcdCopyRHInterruptData(pHCDData,
                     (UINT32)(USB_SYNOPSYSHCD_RH_MASK_VALUE << (pSetup->wIndex -1)));
            break;
            }
        case USB_HUB_FSEL_PORT_POWER:/* Port Power */
            {
            uTempReg = USB_SYNOPSYSHCD_READ32_REG(pHCDData, USB_SYNOPSYSHCD_HPRT);
            uTempReg &= ~USB_SYNOPSYSHCD_HPRT_PRTENA;
            uTempReg |= USB_SYNOPSYSHCD_HPRT_PRTPWR;
            USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                        USB_SYNOPSYSHCD_HPRT,
                                        uTempReg);

            /* Update the status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
#if 0 /* If we need to test we can open this */
        case USB_SYNOPSYSHCD_RH_PORT_TEST: /* Port test */
            {
            /* Update the URB status */

            pURB->nStatus = USBHST_SUCCESS;
            break;
            }
#endif
        default :
            {
            USB_SHCD_ERR("Invalid request\n", 0, 0, 0, 0, 0, 0);

            /* Update the URB status */

            pURB->nStatus = USBHST_INVALID_REQUEST;
            break;
            }
        }/* End of switch */

    return Status;
    }

/* End of file */
