/* usbSynopsysHcdInitExit_rpi_3.c - Synopsys HCD initialization routine */

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

This file contains the initialization and uninitialization routines
provided by the Synopsys USB Host Controller Driver.
*/

/* includes */

#include <vxWorks.h>
#include <string.h>
#include <rebootLib.h>
#include <spinLockLib.h>

#include <usbOsal.h>
#include <usbHst.h>
#include <usbHcdInstr.h>
#include "usbSynopsysHcdDataStructures_rpi_3.h"
#include "usbSynopsysHcdEventHandler_rpi_3.h"
#include "usbSynopsysHcdInterfaces_rpi_3.h"
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"
#include "usbSynopsysHcdRegisterInfo_rpi_3.h"
#include "usbSynopsysHcdUtil_rpi_3.h"

/* defines */

#define USB_SYNOPSYSHCD_RH_DOWN_PORTS          1

/* Name to register vxBus*/
#define USB_SHCI_HUB_NAME "vxbUsbSynopsysHciHub"
#define USB_SHCI_PLB_NAME "vxbPlbUsbSynopsysHci"
#define USB_SHCI_PCI_NAME "vxbPciUsbSynopsysHci"

/*
 * VxBus GEN1 and GEN2 selection module
 */

#include "vxbUsbSynopsysHcd_rpi_3.c"
#ifdef _WRS_CONFIG_FDT
#include "vxbFdtUsbSynopsysHcd_rpi_3.c"
#endif /* _WRS_CONFIG_FDT */

/* forward declarations */

IMPORT VOID usbSynopsysHcdFlushRequest (pUSB_SYNOPSYSHCD_DATA pHCDData);

/* locals */

LOCAL VOID usbSynopsysHcdHostDataUnInit(pUSB_SYNOPSYSHCD_DATA pHCDData);

/* globals */

int usbSynopsysHcdDisableHC(int startType);     /* function to be attached as
                                                 * reboot hook
                                                 */


/* To hold the array of pointers to the HCD maintained data structures */

pUSB_SYNOPSYSHCD_DATA * g_pSynopsysHCDData = NULL;

/* To hold the handle returned by the USBD during HC driver registration */

UINT32  g_SynopsysHCDHandle = 0;

/* Number of host controllers present in the system */

UINT32  g_SynopsysHCDControllerCount = 0;

/* Event used for synchronizing the access of the free lists */

OS_EVENT_ID g_SynopsysHcdListAccessEvent = NULL;

/* To hold the array of pointers to the HCD vxBus device objects */
VXB_DEV_ID g_SynopsysHCDVxbDevId[USB_MAX_SYNOPSYSHCI_COUNT];

/*******************************************************************************
*
* usbSynopsysHcdInstantiate - instantiate the Synopsys USB HCD
*
* This routine instantiates the Synopsys USB Host Controller Driver and allows
* the Synopsys USB Host Controller driver to be included with the vxWorks image
* and not be registered with vxBus. Synopsys USB Host Controller devices will
* remain orphan devices until the usbSynopsysHcdInit() routine is called. This
* supports the INCLUDE_SYNOPSYSHCI behaviour of previous vxWorks releases.
*
* The routine itself does nothing.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void usbSynopsysHcdInstantiate (void)
    {
    return;
    }

/*******************************************************************************
*
* usbSynopsysHcdRpi3Init - initialize the Raspberry Pi 3 Synopsys USB Host
* Controller Driver
*
* This routine intializes the Synopsys USB Host Controller data structures.
* This routine is executed prior to vxBus device connect to initialize
* data structures expected by the device initialization.
*
* The USBD must be initialized prior to calling this routine.
* In this routine the book-keeping variables for the Synopsys USB Host
* Controller Driver are initialized.
*
* RETURNS: OK, or ERROR if the initialization fails
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdRpi3Init (void)
    {
    USBHST_STATUS               status = USBHST_FAILURE;/* To hold the status */
    USBHST_HC_DRIVER            g_pSYNOPSYSHCDriverInfo;/* Structure to hold
                                                         * Synopsys HCI driver
                                                         * informations
                                                         */
    UINT8                       index = 0;
    
    /*
     * Check whether the globals are initialized - This can happen if this
     * routine is called more than once.
     */

    if (g_SynopsysHCDHandle != 0)
        {
        USB_SHCD_ERR("SYNOPSYSHCD already initialized\n", 0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    if (g_pSynopsysHCDData != NULL)
        {
        USB_SHCD_ERR("SYNOPSYSHCD already initialized\n", 0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    /* Initialize the global array */

    g_pSynopsysHCDData = (pUSB_SYNOPSYSHCD_DATA *)OS_MALLOC
                         (sizeof(pUSB_SYNOPSYSHCD_DATA) *
                          USB_MAX_SYNOPSYSHCI_COUNT);


    /* Check if memory allocation is successful */

    if (NULL == g_pSynopsysHCDData)
        {
        USB_SHCD_ERR("location failed for g_pSynopsysHCDData\n", 0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    /* Reset the global array */

    OS_MEMSET (g_pSynopsysHCDData, 0,
               sizeof(pUSB_SYNOPSYSHCD_DATA) * USB_MAX_SYNOPSYSHCI_COUNT);

    /* Create the event used for synchronising the free list accesses */

    g_SynopsysHcdListAccessEvent = OS_CREATE_EVENT(OS_EVENT_SIGNALED);

    /* Check if the event is created successfully */

    if (NULL == g_SynopsysHcdListAccessEvent)
        {
        USB_SHCD_ERR("Error in creating the list event\n", 0, 0, 0, 0, 0, 0);

        /* Free the global array */

        OS_FREE (g_pSynopsysHCDData);

        g_pSynopsysHCDData = NULL;

        return ERROR;
        }

    /* Initialize the members of the data structure */

    OS_MEMSET(&g_pSYNOPSYSHCDriverInfo, 0, sizeof(USBHST_HC_DRIVER));

    /* Populate the members of the HC Driver data structure - start */
    /* Function to retrieve the frame number */

    g_pSYNOPSYSHCDriverInfo.getFrameNumber = usbSynopsysHcdGetFrameNumber;

    /* Function to change the frame interval */

    g_pSYNOPSYSHCDriverInfo.setBitRate = usbSynopsysHcdSetBitRate;

    /* Function to check whether bandwidth is available */

    g_pSYNOPSYSHCDriverInfo.isBandwidthAvailable = usbSynopsysHcdIsBandwidthAvailable;

    /* Function to create a pipe */

    g_pSYNOPSYSHCDriverInfo.createPipe = usbSynopsysHcdCreatePipe;

    /* Function to modify the default pipe */

    g_pSYNOPSYSHCDriverInfo.modifyDefaultPipe = usbSynopsysHcdModifyDefaultPipe;

    /* Function to delete the pipe */

    g_pSYNOPSYSHCDriverInfo.deletePipe = usbSynopsysHcdDeletePipe;

    /* Function to check if the request is pending */

    g_pSYNOPSYSHCDriverInfo.isRequestPending = usbSynopsysHcdIsRequestPending;

    /* Function to submit an URB */

    g_pSYNOPSYSHCDriverInfo.submitURB = usbSynopsysHcdSubmitURB;

    /* Function to cancel an URB */

    g_pSYNOPSYSHCDriverInfo.cancelURB = usbSynopsysHcdCancelURB;

    /* Function to control the pipe characteristics */

    g_pSYNOPSYSHCDriverInfo.pipeControl = usbSynopsysHcdPipeControl;

    /* Function to submit a clear tt request complete */

    g_pSYNOPSYSHCDriverInfo.clearTTRequestComplete = NULL;

    /* Function to submit a reset tt request complete */

    g_pSYNOPSYSHCDriverInfo.resetTTRequestComplete = NULL;

    /* Populate the members of the HC Driver data structure - End */

    /*
     * Register the HCD with the USBD. We also pass the bus id in this function
     * This is to register Synopsys HCI driver with vxBus as a bus type.
     * After the registration is done we get a handle "g_SynopsysHCDHandle".
     * This handle is used for all subsequent communication of Synopsys HCI
     * driver with USBD.
     */

    status = usbHstHCDRegister(&g_pSYNOPSYSHCDriverInfo,
                               &g_SynopsysHCDHandle,
                               NULL,
                               0
                               );

    /* Check whether the registration is successful */

    if (USBHST_SUCCESS != status)
        {
        USB_SHCD_ERR("Error in registering the HCD \n", 0, 0, 0, 0, 0, 0);

        /* Destroy the event */

        (void)OS_DESTROY_EVENT(g_SynopsysHcdListAccessEvent);

        g_SynopsysHcdListAccessEvent = NULL;

        /* Free the global array */

        OS_FREE (g_pSynopsysHCDData);

        g_pSynopsysHCDData = NULL;

        return ERROR;
        }

    for (index = 0; index < USB_MAX_SYNOPSYSHCI_COUNT; index++)
        {
        if (g_SynopsysHCDVxbDevId[index] != NULL)
            (void)VXB_DEV_ATTACH(g_SynopsysHCDVxbDevId[index]);
        }

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdRpi3Exit - uninitialize the Raspberry Pi 3 Synopsys HCI 
* host controller
*
* This routine uninitializes the Synopsys USB Host Controller Driver and
* detaches it from the usbd interface layer.
*
* RETURNS: OK, or ERROR if there is an error during HCD uninitialization.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdRpi3Exit(void)
    {
    USBHST_STATUS Status = USBHST_FAILURE;
    pUSB_SYNOPSYSHCD_DATA      pHCDData = NULL;
    UINT8                      index = 0;

    /* Flush request to terminate all the transfer */

    for (index = 0; index < USB_MAX_SYNOPSYSHCI_COUNT; index++)
        {

        /* Extract the pointer from the global array */

        pHCDData = g_pSynopsysHCDData[index];

        /* Check if the pointer is valid */

        if (NULL == pHCDData)
            break;

        /* Flush the request */

        usbSynopsysHcdFlushRequest(pHCDData);
        
        /* Shutdown the HCD */
        if (pHCDData->pDev)
            {
            (void)VXB_DEV_DETACH(pHCDData->pDev);
            g_SynopsysHCDVxbDevId[index] = pHCDData->pDev;
            }
        }

    /* Deregister the HCD from USBD */

    Status = usbHstHCDDeregister(g_SynopsysHCDHandle);

    /* Check if HCD is deregistered successfully */

    if (USBHST_SUCCESS != Status)
        {
        USB_SHCD_ERR("Failure in deregistering the HCD\n",
                     0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    g_SynopsysHCDHandle = 0;

    /* Destroy the event used for synchronisation of the free list */

    (void)OS_DESTROY_EVENT(g_SynopsysHcdListAccessEvent);

    g_SynopsysHcdListAccessEvent = NULL;

    /* Free the memory allocated for the global data structure */

    OS_FREE(g_pSynopsysHCDData);

    g_pSynopsysHCDData = NULL;

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdDisableHC - called on a reboot to disable the host controller
*
* This routine is called on a warm reboot to disable the host controller by
* the BSP.
*
* RETURNS: 0, always
*
* ERRNO: N/A
*
* \NOMANUAL
*/

int usbSynopsysHcdDisableHC
    (
    int startType
    )
    {

    /* Pointer to the HCD data structure */

    pUSB_SYNOPSYSHCD_DATA      pHCDData = NULL;
    UINT8                      index = 0;

    if ((0 == g_SynopsysHCDControllerCount) ||
        (NULL == g_SynopsysHcdListAccessEvent) ||
        (NULL == g_pSynopsysHCDData))
        {
        return 0;
        }

    /* This loop releases the resources for all the host controllers present */

    for (index = 0; index < USB_MAX_SYNOPSYSHCI_COUNT; index++)
        {
        /* Extract the pointer from the global array */

        pHCDData = g_pSynopsysHCDData[index];

        /* Check if the pointer is valid */

        if (NULL == pHCDData)
            continue;

        /* Disable all interrupts */

        USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                    USB_SYNOPSYSHCD_GINTMSK,
                                    USB_SYNOPSYSHCD_INTERRUPT_MASK);

        /* Turn off the Vbus power */

        USB_SYNOPSYSHCD_CLEARBITS32_REG(pHCDData,
                                      USB_SYNOPSYSHCD_HPRT,
                                      USB_SYNOPSYSHCD_HPRT_PRTPWR);

        /* Call the function to unregister the interrupt line */

        (void) usbSynopsysHcdHostBusUnInit (pHCDData);

        }

    return 0;
    }

/*******************************************************************************
*
* usbSynopsysHcdHostDataInit - initialize USB_SYNOPSYSHCD_DATA structure.
*
* This routine initializes the USB_SYNOPSYSHCD_DATA structure.
*
* RETURNS: OK, or ERROR if USB_SYNOPSYSHCD_DATA initialization is unsuccessful.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdHostDataInit
    (
    pUSB_SYNOPSYSHCD_DATA            pHCDData /* Pointer to HCD block */
    )
    {
    UCHAR ThreadName[20]; /* To hold the name of the interrupt thread */
    UCHAR TransferThreadName[20]; /* To hold the name of the transfer thread */
    UINT32 uHubStatusChangeSize = 0; /* To hold hub status change data size */

    /* Check the validity of the parameter */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Parameter is not valid\n", 0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    /* Init RootHub Data */

    pHCDData->RHData.uNumPorts = pHCDData->usbNumPorts;

    /* Check if the number of ports is valid */

    if (0 == pHCDData->RHData.uNumPorts)
        {
        USB_SHCD_ERR("uNumPorts is 0, reject!\n", 0, 0, 0, 0, 0, 0);
        return ERROR;
        }

    /* Allocate memory for the Root hub port status */

    pHCDData->RHData.pPortStatus = (UCHAR *)
                                    OS_MALLOC(pHCDData->RHData.uNumPorts
                                    * USB_SYNOPSYSHCD_HUBSTATUS_SIZE);

    /* Check if memory allocation is successful */

    if (NULL == pHCDData->RHData.pPortStatus)
        {
        USB_SHCD_ERR("memory not allocated for port status\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);
        return ERROR;
        }

    /* Initialize the fields of the port status register */

    OS_MEMSET(pHCDData->RHData.pPortStatus, 0,
              pHCDData->RHData.uNumPorts *
              USB_SYNOPSYSHCD_HUBSTATUS_SIZE);

    /*
     * The interrupt transfer data is of the following format
     * D0 - Holds the status change information of the hub
     * D1 - Holds the status change information of the Port 1
     *
     * Dn - holds the status change information of the Port n
     * So if the number of downstream ports is N, the size of interrupt
     * transfer data would be N + 1 bits.
     */

    uHubStatusChangeSize = (pHCDData->RHData.uNumPorts + 1)/ 8;
    if (0 != ((pHCDData->RHData.uNumPorts + 1) % 8))
        {
        uHubStatusChangeSize = uHubStatusChangeSize + 1;
        }

    /* Allocate memory for the interrupt transfer data */

    pHCDData->RHData.pHubInterruptData = OS_MALLOC(uHubStatusChangeSize);

    /* Check if memory allocation is successful */

    if (NULL == pHCDData->RHData.pHubInterruptData)
        {
        USB_SHCD_ERR("memory not allocated for interrupt transfer data\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);

        return ERROR;
        }

    /* Initialize the interrupt data */

    OS_MEMSET(pHCDData->RHData.pHubInterruptData, 0, uHubStatusChangeSize);

    /* Copy the size of the interrupt data */

    pHCDData->RHData.uSizeInterruptData = uHubStatusChangeSize;

    /* Create the default pipe */

    pHCDData->pDefaultPipe = usbSynopsysHcdNewPipe();

    /* Check if default pipe is created successfully */

    if (NULL == pHCDData->pDefaultPipe)
        {
        USB_SHCD_ERR("memory not allocated for the default pipe\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);

        return ERROR;
        }

    /* Create the event which is used for signalling the thread on interrupt */

    pHCDData->interruptEvent = OS_CREATE_EVENT(OS_EVENT_NON_SIGNALED);

    /* Check if the event is created successfully */

    if (NULL == pHCDData->interruptEvent)
        {
        USB_SHCD_ERR("signal event not created\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);
        return ERROR;
        }

    /*
     * Create message queue which is used for transfer
     * message to transfer thread
     */

    pHCDData->transferThreadMsgQ = msgQCreate(USB_SYNOPSYSHCD_MSGQ_COUNT,
                                              sizeof(USB_SYNOPSYSHCD_TRANSFER_TASK_INFO),
                                              MSG_Q_FIFO);

    if (NULL == pHCDData->transferThreadMsgQ)
        {
        USB_SHCD_ERR("message queue not created\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);

        return ERROR;
        }

    /* Create the event used for synchronisation of the requests */

    pHCDData->RequestSynchEventID = semMCreate(SEM_Q_PRIORITY     |
                                               SEM_INVERSION_SAFE |
                                               SEM_DELETE_SAFE);

    /* Check if the event is created successfully */

    if (NULL == pHCDData->RequestSynchEventID)
        {
        USB_SHCD_ERR("synch event not created\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);

        return ERROR;
        }

    /* Init the pipe list */

    lstInit(&(pHCDData->pipeList));

    /* Init the schedule list for transfer data */

    lstInit(&(pHCDData->periodicReqList));

    lstInit(&(pHCDData->periodicReqReadyList));

    lstInit(&(pHCDData->nonPeriodicReqList));

    lstInit(&(pHCDData->nonPeriodicReqReadyList));

    /* Init the IdleDmaChannelMap, a bit of 1 means idle */

    pHCDData->uIdleDmaChannelMap = 0xFFFFFFFF;

    /* Assign an unique name for the interrupt handler thread */

    (void)snprintf((char*)ThreadName, 20, "SynoISR%d", pHCDData->uBusIndex);

    pHCDData->dmaChannelMutex = semMCreate(SEM_Q_PRIORITY |
                                           SEM_INVERSION_SAFE |
                                           SEM_DELETE_SAFE);

    if (pHCDData->dmaChannelMutex == NULL)
        {
        USB_SHCD_ERR("usbSynopsysHcdHostDataInit(): "
                     "creat dmaChannelMutex failed\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Create the interrupt handler thread for handling the interrupts */

    pHCDData->intHandlerThread = OS_CREATE_THREAD((char *)ThreadName,
                                                  USB_SYNOPSYSHCD_INT_THREAD_PRIORITY,
                                                  usbSynopsysHcdInterruptHandler,
                                                  pHCDData);

    /* Check whether the thread creation is successful */

    if (OS_THREAD_FAILURE == pHCDData->intHandlerThread)
        {
        USB_SHCD_ERR("interrupt handler thread is not created\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);

        return ERROR;
        }


    /* Assign an unique name for the transfer handler thread */

    (void)snprintf((char*)TransferThreadName, 20, "SynoTran%d", pHCDData->uBusIndex);

    /*
     * Create the transfer handler thread for handling the message sended by
     * the interrupt thread
     */

    pHCDData->transferThread = OS_CREATE_THREAD((char *)TransferThreadName,
                                                 USB_SYNOPSYSHCD_INT_THREAD_PRIORITY,
                                                 usbSynopsysHcdTransferHandler,
                                                 pHCDData);

    /* Check whether the thread creation is successful */

    if (OS_THREAD_FAILURE == pHCDData->transferThread)
        {
        USB_SHCD_ERR("transfer handler thread is not created\n",
                     0, 0, 0, 0, 0, 0);

        usbSynopsysHcdHostDataUnInit(pHCDData);

        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdHostDataUnInit - uninitialize the USB_SYNOPSYSHCD_DATA structure
*
* This routine uninitializes the USB_SYNOPSYSHCD_DATA structure.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usbSynopsysHcdHostDataUnInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData /* Pointer to HCD block */
    )
    {
    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("invalid parameter\n", 0, 0, 0, 0, 0, 0);
        return;
        }

    /* Destroy the thread created for handling the interrupts */

    if (OS_THREAD_FAILURE != pHCDData->intHandlerThread
        && 0 != pHCDData->intHandlerThread)
        {
        (void) OS_DESTROY_THREAD(pHCDData->intHandlerThread);
        pHCDData->intHandlerThread = OS_THREAD_FAILURE;
        }

    /* Destroy the signalling event */

    if (NULL != pHCDData->interruptEvent)
        {
        (void)OS_DESTROY_EVENT(pHCDData->interruptEvent);
        pHCDData->interruptEvent = NULL;
        }

    if (NULL != pHCDData->dmaChannelMutex)
        {
        (void)OS_DESTROY_EVENT(pHCDData->dmaChannelMutex);
        pHCDData->dmaChannelMutex = NULL;
        }

    /* Destroy the thread created for handling the message */

    if (0 != pHCDData->transferThread
        && OS_THREAD_FAILURE != pHCDData->transferThread)
        {
        (void) OS_DESTROY_THREAD(pHCDData->transferThread);
        pHCDData->transferThread = OS_THREAD_FAILURE;
        }

    /* Destroy the MsgQ */

    if (pHCDData->transferThreadMsgQ)
        {
        (void) msgQDelete (pHCDData->transferThreadMsgQ);
        pHCDData->transferThreadMsgQ = NULL;
        }

    /* Destroy the synchronization event */

    if (NULL != pHCDData->RequestSynchEventID)
        {
        if (OK == OS_WAIT_FOR_EVENT(pHCDData->RequestSynchEventID, WAIT_FOREVER))
            {
            (void)OS_DESTROY_EVENT(pHCDData->RequestSynchEventID);
            pHCDData->RequestSynchEventID = NULL;
            }
        else
            {
            USB_SHCD_ERR("Take pHCDData->RequestSynchEventID failed. \n",
                         1, 2, 3, 4, 5, 6);
            }
        }

    /* Free memory allocated for the default pipe */

    if (NULL != pHCDData->pDefaultPipe)
        {
        usbSynopsysHcdDestroyPipe(pHCDData->pDefaultPipe);
        pHCDData->pDefaultPipe = NULL;
        }

    /* Free memory allocated for the interrupt data */

    if (NULL != pHCDData->RHData.pHubInterruptData)
        {
        OS_FREE(pHCDData->RHData.pHubInterruptData);
        pHCDData->RHData.pHubInterruptData = NULL;
        }

    /* Free memory allocated for the port status */

    if (NULL != pHCDData->RHData.pPortStatus)
        {
        OS_FREE(pHCDData->RHData.pPortStatus);
        pHCDData->RHData.pPortStatus = NULL;
        }
    return;
    }

/* End of file */
