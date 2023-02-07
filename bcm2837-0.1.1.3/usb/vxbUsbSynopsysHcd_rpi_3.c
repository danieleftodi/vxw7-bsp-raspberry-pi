/* vxbUsbSynopsysHcd_rpi_3.c - USB SYNOPSYSHCD Driver Inferface for VxBUS GEN2 */

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

This file provides functions for Synopsys HCD Driver.

INCLUDE FILES: subsys/timer/vxbTimerLib.h, subsys/int/vxbIntLib.h, verrnoLib.h,
vxbUsbSynopsysHcd_rpi_3.h and usbSynopsysHcdHardwareAccess_rpi_3.h
*/

/* includes */

#include <subsys/timer/vxbTimerLib.h>
#include <subsys/int/vxbIntLib.h>
#include <errnoLib.h>
#include <vxbUsbSynopsysHcd_rpi_3.h>
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"

/* defines */

/* locals */

LOCAL STATUS usbSynopsysHcdHostBusInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );

LOCAL VOID usbSynopsysHcdHostBusUnInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );

LOCAL VOID usbSynopsysHcdHostDataUnInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData
    );

STATUS usbSynopsysHcdDetach
    (
    struct vxbDev * pDev
    );

/* externs */

IMPORT VOID usbSynopsysHcdDestroyInterruptTree
    (
    pUSB_SYNOPSYSHCD_DATA    pHCDData
    );

IMPORT BOOLEAN usbSynopsysHcdCreateInterruptTree
    (
    pUSB_SYNOPSYSHCD_DATA    pHCDData
    );

IMPORT STATUS usbSynopsysHcdHostDataInit (pUSB_SYNOPSYSHCD_DATA pHCDData);

VXB_DEVMETHOD_DECL (vxbClkMgrPeriphEnable)
typedef STATUS (*vxbClkMgrPeriphEnable_t) (UINT32, BOOL);

VXB_DEVMETHOD_DECL (vxbRstMgrPeriphUSBReset)
typedef STATUS (*vxbRstMgrPeriphUSBReset_t) (UINT32);

/* SYNOPSYSHCD Defalut configure */

USB_SYNOPSYSHCD_CFG synopsysHcdDefaultCfg =
    {
    NULL,                  /* pPerInitHook */
    NULL,                  /* pPostResetHook */
    NULL,                  /* pUnInitHook */
    NULL,                  /* pDescSwap */
    NULL,                  /* pUsbSwap */
    NULL,                  /* pRegSwap */
    0,                     /* uPhyBaseAddr */
    0,                     /* uPlatformType */
    0,                     /* uRegOffset */
    FALSE,                 /* bFixupPortNumber */
    FALSE,                 /* bDescBigEndian */
    FALSE,                 /* bRegBigEndian */
    FALSE,                 /* bIntEachTD */
    TRUE,                  /* bHasCompanion */
    0,                     /* uDevClass */
    NULL,                  /* pPhyDev */
    USB_PHY_TYPE_ULPI,     /* phyType */
    NULL                   /* pExtData */
    };

/* Number of host controllers present in the system */

extern UINT32  g_SynopsysHCDControllerCount;

/*
 * Array of Spinlocks for the Synopsys USB host Controllers. The array will be
 * allocated in the usbShcdInit once the maxShciCount is known.
 */

spinlockIsr_t spinLockIsrSynopsysHcd[USB_MAX_SYNOPSYSHCI_COUNT];

/*******************************************************************************
*
* usbSynopsysHcdDataIndexRelease - release the index of USB SYNOPSYSHCD data
*
* This routine is used to release the index USB SYNOPSYSHCD data.
*
* RETURNS: OK with a valid usb SYNOPSYSHCD data, or ERROR otherwise.
*/

STATUS usbSynopsysHcdDataIndexRelease
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* HCD data */
    )
    {
    if (pHCDData == NULL)
        {
        USB_SHCD_ERR(
            "usbSynopsysHcdDataIndexRelease - Invalid parameter\n",
            0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    (void)OS_WAIT_FOR_EVENT (g_SynopsysHcdListAccessEvent, WAIT_FOREVER);

    /* Incremnet the global counter */

    g_pSynopsysHCDData[pHCDData->uBusIndex] = NULL;
    g_SynopsysHCDControllerCount --;

    /* Initialize the ISR Spinlock */

    SPIN_LOCK_ISR_INIT(&spinLockIsrSynopsysHcd[pHCDData->uBusIndex], 0);

    (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdDataIndexReserve - reserve the index for USB SYNOPSYSHCD data
*
* This routine is used to reserve for the index for USB SYNOPSYSHCD data.
*
* RETURNS: OK with a valid usb SYNOPSYSHCD data, or ERROR otherwise.
*/

STATUS usbSynopsysHcdDataIndexReserve
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* HCD data */
    )
    {
    UINT8 uIndex;

    if (pHCDData == NULL)
        {
        USB_SHCD_ERR(
            "usbSynopsysHcdDataFreeIndexGet - Invalid parameter\n",
            0, 0, 0, 0, 0, 0);

        return ERROR;
        }

    (void)OS_WAIT_FOR_EVENT (g_SynopsysHcdListAccessEvent, WAIT_FOREVER);

    if (g_SynopsysHCDControllerCount == USB_MAX_SYNOPSYSHCI_COUNT)
        {
        USB_SHCD_ERR(
            "usbSynopsysHcdDataFreeIndexGet - Out of resources \n",
            0, 0, 0, 0, 0, 0);

        (void)errnoSet(USBHST_INSUFFICIENT_RESOURCE);

        (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);

        return ERROR;
        }

    /* Get bus index */

    for (uIndex = 0; uIndex < USB_MAX_SYNOPSYSHCI_COUNT; uIndex++)
        {
        if (g_pSynopsysHCDData[uIndex] == NULL)
            break;
        }

    if (uIndex == USB_MAX_SYNOPSYSHCI_COUNT)
        {
        USB_SHCD_ERR("usbSynopsysHcdDataFreeIndexGet - "
                     "g_pSynopsysHCDData is full\n",
                     0, 0, 0, 0, 0, 0);

        (void)errnoSet(USBHST_INSUFFICIENT_RESOURCE);

        (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);

        return ERROR;
        }

    /* Incremnet the global counter */
    g_pSynopsysHCDData[uIndex] = pHCDData;
    g_SynopsysHCDControllerCount++;

    /* Initialize the ISR Spinlock */

    SPIN_LOCK_ISR_INIT(&spinLockIsrSynopsysHcd[uIndex], 0);

    (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);

    pHCDData->uBusIndex = uIndex;

    return OK;
    }
    /* End of function usbSynopsysHcdDataIndexReserve() */

/*******************************************************************************
*
* usbSynopsysHcdStop - initialize SYNOPSYSHCD_DATA structure
*
* This routine initializes the SYNOPSYSHCD_DATA structure.
*
* RETURNS: TURE, or FALSE if SYNOPSYSHCD_DATA initialization is unsuccessful.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdStop
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* HCD data */
    )
    {
    USBHST_STATUS hStatus;

    if ((NULL == pHCDData) ||
        (NULL == pHCDData->pDev))
        {
        USB_SHCD_ERR("usbSynopsysHcdStop - invalid parameter\n",
            0, 0, 0, 0, 0, 0);

        return ERROR;
        }

#ifdef USB_SHCD_POLLING_MODE

    /* Destroy the thread created for handling the interrupts */

    if ((synopsysHcdPollingTh[pHCDData->uBusIndex] != OS_THREAD_FAILURE)&&
        (synopsysHcdPollingTh[pHCDData->uBusIndex] != 0))
        {
        OS_DESTROY_THREAD(synopsysHcdPollingTh[pHCDData->uBusIndex]);

        synopsysHcdPollingTh[pHCDData->uBusIndex] = OS_THREAD_FAILURE;
        }
#else
    (void) vxbIntDisable(pHCDData->pDev, pHCDData->pIntRes);
    (void) vxbIntDisconnect(pHCDData->pDev, pHCDData->pIntRes);
#endif

    /* Reset the Host Controller */

    usbSynopsysHCDResetCore(pHCDData);

    /* We can not respond to interrupts */

    pHCDData->isrMagic = USB_SYNOPSYSHCD_MAGIC_DEAD;

    /* call the function to de-register the bus */

    hStatus = usbHstBusDeregister(g_SynopsysHCDHandle,
                                 pHCDData->uBusIndex,
                                 (ULONG)pHCDData->pDefaultPipe);

    /* Check if the bus is deregistered successfully */

    if (USBHST_SUCCESS != hStatus)
        {
        USB_SHCD_ERR("usbHcdSynopsysHcdDeviceRemove - "
            "Failure in deregistering the bus\n",
            0, 0, 0, 0, 0, 0);
        }

    (void)OS_WAIT_FOR_EVENT (g_SynopsysHcdListAccessEvent, WAIT_FOREVER);

    g_pSynopsysHCDData[pHCDData->uBusIndex] = NULL;

    /* Decrement the global counter by 1 */

    g_SynopsysHCDControllerCount--;

    (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdStart - initialize SYNOPSYSHCD_DATA structure
*
* This routine initializes the SYNOPSYSHCD_DATA structure.
*
* RETURNS: TURE, or FALSE if SYNOPSYSHCD_DATA initialization is unsuccessful.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS usbSynopsysHcdStart
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* HCD data */
    )
    {
    struct vxbDev * pDev = NULL;
    UINT32          uCount;
    STATUS          status = ERROR;

    INT32           offset       = 0;
    UINT32          phyaddr      = 0;
    void          * virtAddr     = NULL;
    void          * prop         = NULL;
    UINT32        * eccGrpReg    = NULL;
    UINT32          eccGrpRegVal = 0;

    if ((NULL == pHCDData) ||
        (NULL == (pDev = pHCDData->pDev)))
        {
        USB_SHCD_ERR("usbSynopsysHcdStart - invalid parameter\n",
            0, 0, 0, 0, 0, 0);
        goto exit;
        }
        
    if (pHCDData->uPlatformType != USB_SYNOPSYSHCI_PLATFORM_RPI_3)
        {

        /* enable ECC on the USB RAM */

        offset = vxFdtPathOffset("/alt_soc/sysmgr");
        if (offset <= 0)
            goto exit;

        prop = (UINT32*) vxFdtPropGet(offset, "reg", NULL);
        if (prop == NULL)
            goto exit;

        phyaddr = vxFdt32ToCpu(*(UINT32*)prop);

        virtAddr = pmapGlobalMap ((PHYS_ADDR) phyaddr, (size_t) 0x1000,
                                   MMU_ATTR_SUP_RW | MMU_ATTR_CACHE_OFF |
                                   MMU_ATTR_CACHE_GUARDED);
        if (virtAddr == PMAP_FAILED)
            goto exit;

        eccGrpReg = (UINT32*)((UINT8*)virtAddr +
                               SYSMGR_ECCGRP_USB(pHCDData->usbDevNum));

        eccGrpRegVal = SWAP32(vxbRead32(pHCDData->pRegAccessHandle, eccGrpReg));
        eccGrpRegVal |= SYSMGR_ECCGRP_USB_RAM_ECC_EN;

        vxbWrite32(pHCDData->pRegAccessHandle, eccGrpReg, SWAP32(eccGrpRegVal));
        }

    /* Enable USB PHY Dev */

    if (pHCDData->pPhyDev)
        {
        if (ERROR == vxbUsbPhyEnable(pHCDData->pPhyDev))
            {
            USB_SHCD_WARN("usbSynopsysHcdDetach - Diable PHY fail \n",
                          1, 2, 3, 4, 5, 6);
            goto exit;
            }
        }

    /* Take the mutex */

    (void)OS_WAIT_FOR_EVENT (g_SynopsysHcdListAccessEvent, WAIT_FOREVER);

    if (USB_MAX_SYNOPSYSHCI_COUNT == g_SynopsysHCDControllerCount)
        {
        USB_SHCD_ERR("Out of resources \n", 0, 0, 0, 0, 0, 0);

        status = USBHST_INSUFFICIENT_RESOURCE;
        (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);
        goto exit;
        }

    /* Update the unit number with g_SynopsysHCDControllerCount */

    for (uCount = 0; uCount < USB_MAX_SYNOPSYSHCI_COUNT; uCount ++)
        {
        if (g_pSynopsysHCDData[uCount]== NULL)
            break;
        }

    if (uCount == USB_MAX_SYNOPSYSHCI_COUNT)
        {
        USB_SHCD_ERR("g_pSynopsysHCDData is full\n",
            0, 0, 0, 0, 0, 0);

        status = USBHST_INSUFFICIENT_RESOURCE;

        (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);
        goto exit;
        }

    /* Initialize the ISR Spinlock */

    SPIN_LOCK_ISR_INIT(&spinLockIsrSynopsysHcd[uCount], 0);

    /*
     * Call the bus specific initialization function to initialize the
     * host controllers in the system
     */

    status = usbSynopsysHcdHostBusInit(pHCDData);
    if (OK != status)
        {
        USB_SHCD_ERR("usbSynopsysHcdHostBusInit failed\n", 0, 0, 0, 0, 0, 0);

        /* Reset the global array */

        g_pSynopsysHCDData[uCount] = NULL;
        status = USBHST_FAILURE;
        goto exit;
        }

    /* Initialize the Host Controller data structure */

    status = usbSynopsysHcdHostDataInit(pHCDData);
    if (OK != status)
        {
        USB_SHCD_ERR("usbSynopsysHcdHostDataInit failed\n", 0, 0, 0, 0, 0, 0);

        /* Call the uninitialization function of the HC bus */

        (void) usbSynopsysHcdHostBusUnInit(pHCDData);
        status = USBHST_FAILURE;
        goto exit;
        }

    /*
     * After we call vxbIntEnable, we will enter the ISR,
     * So we must make sure the device is OK to handlle the interrupt
     * But usually, the root hub isn't OK now.
     * So we'd better disable the interrupt before vxbIntEnable is called.
     * After the root hub is configured, the interrupt will be enabled
     */

    USB_SYNOPSYSHCD_WRITE32_REG(pHCDData,
                                USB_SYNOPSYSHCD_GINTMSK,
                                USB_SYNOPSYSHCD_INTERRUPT_MASK);

    /* Increase the global counter */

    g_SynopsysHCDControllerCount++;

    /* We can now respond to interrupts */

    pHCDData->isrMagic = USB_SYNOPSYSHCD_MAGIC_ALIVE;

    (void)OS_RELEASE_EVENT (g_SynopsysHcdListAccessEvent);

    /* Connect the interrupt resource */

#ifdef USB_SHCD_POLLING_MODE

    synopsysHcdPollingTh[pHCDData->uBusIndex] = OS_CREATE_THREAD("PollingISR",
                          37,
                          usbShcdPollingISR,
                          (long)(pHCDData->pDev));

    if (synopsysHcdPollingTh[pHCDData->uBusIndex] == OS_THREAD_FAILURE)
        {
        USB_SHCD_ERR(
            "usbSynopsysHcdHostBusInitialize - "
            "Error creating the polling task\n",
            0, 0, 0, 0, 0, 0);

        /* Call the uninitialization function of the HCD data */

        (void)usbSynopsysHcdStop(pHCDData);

        goto exit;
        }
#else
    if (ERROR == vxbIntConnect(pDev,
                                pHCDData->pIntRes,
                                (VOIDFUNCPTR)usbSynopsysHcdISR,
                                (VOID *)(pHCDData)))
        {
        USB_SHCD_ERR ("usbSynopsysHcdStart vxbIntConnect fail %x \r\n",
                      pHCDData->pIntRes, 2, 3,4, 5, 6);
        (void)usbSynopsysHcdStop(pHCDData);
        goto exit;
        }

    if (ERROR == vxbIntEnable(pDev, pHCDData->pIntRes))
       {
       USB_SHCD_ERR ("usbSynopsysHcdStart vxbIntEnable fail\r\n",
                     1, 2, 3,4, 5, 6);
       (void)usbSynopsysHcdStop(pHCDData);
       goto exit;
       }
#endif
    /* Eable the interrupt and Run */

    /* Power on all the root hub ports */

    vxbMsDelay(200);

    /* Register the HCD BUS to USBD */

    status = usbHstBusRegister (g_SynopsysHCDHandle,
                                USBHST_HIGH_SPEED,
                                (ULONG)pHCDData->pDefaultPipe,
                                pDev);

    if (ERROR == status)
        {

        /* Call the uninitialization function of the HCD data */

        (void)usbSynopsysHcdStop(pHCDData);
        goto exit;
        }

    status = OK;
exit:
    if (virtAddr)
        (void)pmapGlobalUnmap(virtAddr, 0x1000);

    if (status != OK)
        (void)usbSynopsysHcdDetach(pHCDData->pDev);

    return status;
    }


/*******************************************************************************
*
* usbSynopsysHcdDataUnConfig - unconfig the SYNOPSYSHCD controller hardware
*
* This routine is used to unconfig the SYNOPSYSHCD controller hardware.
*
* RETURNS: OK, or ERROR if there is anything wrong.
*/

STATUS usbSynopsysHcdDataUnConfig
    (
    struct vxbDev * pDev    /* device data */
    )
    {
    pUSB_SYNOPSYSHCD_DATA pHCDData;

    pHCDData = (pUSB_SYNOPSYSHCD_DATA)vxbDevSoftcGet(pDev);

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("Invalid parameters\n",
                     0, 0, 0, 0, 0, 0);

        (void)errnoSet(USBHST_INVALID_PARAMETER);

        return ERROR;
        }

    if (NULL != pHCDData->pIntRes)
        {
        (void)vxbResourceFree(pDev, pHCDData->pIntRes);
        pHCDData->pIntRes = NULL;
        }

    if (NULL != pHCDData->pRegRes)
        {
        (void)vxbResourceFree(pDev, pHCDData->pRegRes);
        pHCDData->pRegRes = NULL;
        }

    /* Return OK only for a valid index */

    (void)usbSynopsysHcdDataIndexRelease(pHCDData);

    /* Terminate ISR and Transfer threads */

    (void)usbSynopsysHcdHostDataUnInit(pHCDData);

    OSS_FREE(pHCDData);

    /* Reset the softc as NULL */

    vxbDevSoftcSet(pDev, NULL);

    USB_SHCD_DBG ("usbSynopsysHcdDataUnConfig Done\n", 1, 2, 3, 4, 5, 6);

    return OK;
    }


/*******************************************************************************
*
* usbSynopsysHcdDataConfig - config for a USB SYNOPSYSHCD controller hardware
*
* This routine is used to config for a USB SYNOPSYSHCD controller hardware.
*
* RETURNS: OK with a valid usb SYNOPSYSHCD controller, or ERROR otherwise.
*/

LOCAL STATUS usbSynopsysHcdDataConfig
    (
    struct vxbDev * pDev,       /* device data */
    USB_SYNOPSYSHCD_CFG *  pCfg /* HCD configuration */
    )
    {
    pUSB_SYNOPSYSHCD_DATA     pHCDData = NULL;
    VXB_RESOURCE_ADR * pResAdr = NULL;
    VXB_RESOURCE *     pRes = NULL;

    if ((NULL == pDev) || (NULL == pCfg))
        {
        USB_SHCD_ERR("Invalid parameters\n",
                     0, 0, 0, 0, 0, 0);

        (void)errnoSet(USBHST_INVALID_PARAMETER);

        return ERROR;
        }

    /* Alloc the memory for USB_SYNOPSYSHCD_DATA */

    pHCDData = (pUSB_SYNOPSYSHCD_DATA)OSS_CALLOC(sizeof(USB_SYNOPSYSHCD_DATA));

    /* Check if memory allocation is successful */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("usbSynopsysHcdDataConfig - "
                     "USB_SYNOPSYSHCD_DATA allocation failed\n",
                     0, 0, 0, 0, 0, 0);

        (void)errnoSet(USBHST_MEMORY_NOT_ALLOCATED);

        return ERROR;
        }

    vxbDevSoftcSet(pDev, (void *)pHCDData);

    /* Return OK only for a valid index */

    if (ERROR == usbSynopsysHcdDataIndexReserve(pHCDData))
        {
        USB_SHCD_ERR("usbSynopsysHcdDataConfig - "
                     "Too much host controllers\n",
                     0, 0, 0, 0, 0, 0);

        (void)usbSynopsysHcdDataUnConfig(pDev);
        return ERROR;
        }

    /* Only one Address bas as USBBASE Register */

    pRes = vxbResourceAlloc(pDev, VXB_RES_MEMORY, 0);

    if((NULL == pRes) ||
        (NULL == (pResAdr = (VXB_RESOURCE_ADR *)pRes->pRes)))
        {
        USB_SHCD_ERR("usbSynopsysHcdDataConfig(): Get invalid VXB_RES_MEMORY\n",
                     1, 2, 3, 4, 5, 6);

        (void)usbSynopsysHcdDataUnConfig(pDev);
        return ERROR;
        }

    pHCDData->pRegRes = pRes;
    pHCDData->pRegAccessHandle = pResAdr->pHandle;
    pHCDData->regBase = (ULONG)pResAdr->virtual + pCfg->uRegOffset;

    USB_SHCD_DBG("usbSynopsysHcdDataConfig(): "
                 "phyAddr %08x virAddr %08x handle %08x\r\n",
                 pResAdr->start,
                 pResAdr->virtual,
                 pResAdr->pHandle,
                 4, 5, 6);

    /* Interrupt resource */

    pHCDData->pIntRes = vxbResourceAlloc(pDev, VXB_RES_IRQ, 0);

    if (NULL == pHCDData->pIntRes)
        {
        USB_SHCD_ERR("usbSynopsysHcdDataConfig(): Get invalid VXB_RES_IRQ\n",
                     1, 2, 3, 4, 5, 6);

        (void)usbSynopsysHcdDataUnConfig(pDev);
        return ERROR;
        }

    /* Record the configure information */

    pHCDData->pPerInitHook = pCfg->pPerInitHook;
    pHCDData->pPostResetHook = pCfg->pPostResetHook;

    pHCDData->pDescSwap = pCfg->pDescSwap;
    pHCDData->pUsbSwap = pCfg->pUsbSwap;
    pHCDData->pRegSwap = pCfg->pRegSwap;

    pHCDData->phyBaseAddr = pCfg->uPhyBaseAddr;
    pHCDData->uPlatformType = pCfg->uPlatformType;

    pHCDData->pDev = pDev;
    pHCDData->uDevClass = pCfg->uDevClass;
    pHCDData->pPhyDev = pCfg->pPhyDev;
    pHCDData->phyType = pCfg->phyType;

    pHCDData->addressMode64 = pCfg->addressMode64;
    pHCDData->usbNumPorts = pCfg->usbNumPorts;
    pHCDData->hostNumDmaChannels = pCfg->hostNumDmaChannels;
    pHCDData->usbDevNum = pCfg->usbDevNum;
    pHCDData->pRegSwap = pCfg->pRegSwap;
    pHCDData->pDescSwap = pCfg->pDescSwap;
    pHCDData->pCpuToBus = pCfg->pCpuToBus;
    pHCDData->pBusToCpu = pCfg->pBusToCpu;

    if (pHCDData->pRegSwap != NULL)
        {
        pHCDData->pRegAccessHandle = (void *)
            VXB_HANDLE_SWAP ((ULONG)((pHCDData)->pRegAccessHandle));
        }

    USB_SHCD_DBG ("usbSynopsysHcdDataConfig Success\n",
                  1, 2, 3, 4, 5, 6);

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdAttach - Attach the device to the SYNOPSYSHCD driver
*
* This routine is used to attach the SYNOPSYSHCD controller to the SYNOPSYSHCD
* driver.
*
* RETURNS: OK if success, or ERROR when failure.
*
*/

STATUS usbSynopsysHcdAttach
    (
    struct vxbDev * pDev,       /* device data */
    USB_SYNOPSYSHCD_CFG *  pCfg /* HCD configuration */
    )
    {
    STATUS                     status   = ERROR;
    pUSB_SYNOPSYSHCD_DATA      pHCDData = NULL;
    OS_THREAD_ID               hcdStartThread = NULL;

    if(NULL == pDev)
        {
        USB_SHCD_ERR("usbSynopsysHcdAttach - Invalid Parameters \n",
                     1, 2, 3, 4, 5, 6);

        (void) ossStatus (USBHST_INVALID_PARAMETER);

        goto exit;
        }

    if (ERROR == usbSynopsysHcdDataConfig(pDev, pCfg))
        {
        USB_SHCD_ERR("usbSynopsysHcdAttach - "
                     "usbSynopsysHcdDataConfig SYNOPSYSHCD fail\n",
                     1, 2, 3, 4, 5, 6);

        goto exit;
        }

    pHCDData = (pUSB_SYNOPSYSHCD_DATA)vxbDevSoftcGet(pDev);

    /* validate paramters */

    if (NULL == pHCDData)
        {
        USB_SHCD_ERR("usbSynopsysHcdAttach - Invalid Parameters \n",
                     1, 2, 3, 4, 5, 6);

        (void) ossStatus (USBHST_INVALID_PARAMETER);

        goto exit;
        }

    hcdStartThread = OS_CREATE_THREAD((char *)"tSynopsysHcdInit",
                                USB_SYNOPSYSHCD_START_THREAD_PRIORITY,
                                usbSynopsysHcdStart, (long)pHCDData);
    if (hcdStartThread == OS_THREAD_FAILURE)
        goto exit;

    USB_SHCD_DBG("usbSynopsysHcdAttach - Success! \n",
                 1, 2, 3, 4, 5, 6);

    status = OK;
exit:
    if (status != OK)
        {
        if ((NULL != pHCDData) && (pHCDData->pPhyDev))
            {
            (void)vxbUsbPhyDisable(pHCDData->pPhyDev);
            }
        (void)usbSynopsysHcdDataUnConfig(pDev);
        }
    return status;
    }

/*******************************************************************************
*
* usbSynopsysHcdDetach - detach the SYNOPSYSHCD controller
*
* This routine is used to detach the SYNOPSYSHCD controller.
*
* RETURNS: OK, or ERROR when failure.
*
*/

STATUS usbSynopsysHcdDetach
    (
    struct vxbDev * pDev    /* device data */
    )
    {
    pUSB_SYNOPSYSHCD_DATA pHCDData = NULL;

    pHCDData = (pUSB_SYNOPSYSHCD_DATA)vxbDevSoftcGet(pDev);

    /* validate paramters */

    if ((NULL == pHCDData) ||
        (NULL == pDev))
        {
        USB_SHCD_ERR("usbSynopsysHcdDetach - Invalid Parameters \n",
                     1, 2, 3, 4, 5, 6);

        (void) ossStatus (USBHST_INVALID_PARAMETER);

        return ERROR;
        }

    /* Stop the SYNOPSYSHCD */

    if (ERROR == usbSynopsysHcdStop(pHCDData))
        {
        USB_SHCD_WARN("usbSynopsysHcdDetach - Stop SYNOPSYSHCD fail \n",
                      1, 2, 3, 4, 5, 6);
        }

    /* Disable USB PHY Dev */

    if (pHCDData->pPhyDev)
        {
        if (ERROR == vxbUsbPhyDisable(pHCDData->pPhyDev))
            {
            USB_SHCD_WARN("usbSynopsysHcdDetach - Diable PHY fail \n",
                          1, 2, 3, 4, 5, 6);
            }
        }

    /* Unconfig SYNOPSYSHCD data */

    if (ERROR == usbSynopsysHcdDataUnConfig(pDev))
        {
        USB_SHCD_WARN("usbSynopsysHcdDetach - Unconfig SYNOPSYSHCD Data fail\n",
                      1, 2, 3, 4, 5, 6);
        }

    USB_SHCD_DBG("usbSynopsysHcdDetach - Success! \n",
                 1, 2, 3, 4, 5, 6);

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdShutdown - shutdown the SYNOPSYSHCD controller
*
* This routine is used to shutdown the SYNOPSYSHCD controller by VxBus device.
*
* RETURNS: OK
*
*/

STATUS usbSynopsysHcdShutdown
    (
    struct vxbDev * pDev    /* device data */
    )
    {
    pUSB_SYNOPSYSHCD_DATA pHCDData = NULL;

    pHCDData = (pUSB_SYNOPSYSHCD_DATA)vxbDevSoftcGet(pDev);

    /* validate paramters */

    if (NULL == pHCDData)
        {
        USB_SHCD_WARN("usbSynopsysHcdShutdown - Invalid Parameters \n",
                      1, 2, 3, 4, 5, 6);

        (void) ossStatus (USBHST_INVALID_PARAMETER);
        return OK;
        }

#ifndef USB_SHCD_POLLING_MODE
    (void) vxbIntDisable (pHCDData->pDev,pHCDData->pIntRes);
    (void) vxbIntDisconnect (pHCDData->pDev,pHCDData->pIntRes);
#else

    /* Destroy the polling thread */

    if (synopsysHcdPollingTh[pHCDData->uBusIndex] != OS_THREAD_FAILURE
        && synopsysHcdPollingTh[pHCDData->uBusIndex] != 0)
        {
        OS_DESTROY_THREAD (synopsysHcdPollingTh[pHCDData->uBusIndex]);

        synopsysHcdPollingTh[pHCDData->uBusIndex] = OS_THREAD_FAILURE;
        }
#endif
    USB_SHCD_DBG("usbSynopsysHcdShutdown OK\n", 1, 2, 3, 4, 5, 6);

    return OK;
    }

/*******************************************************************************
*
* usbSynopsysHcdHostBusInit - initialize USB Host controller
*
* This routine initializes USB Host controller.
*
* RETURNS: OK or ERROR
*
*/

LOCAL STATUS usbSynopsysHcdHostBusInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData   /* struct usb_synopsyshcd_data */
    )
    {
    STATUS status = ERROR;

    if (pHCDData == NULL)
        goto exit;

    (void)usbSynopsysHCDCoreInit(pHCDData);
    (void)usbSynopsysHCDHostInit(pHCDData);

    pHCDData->initDone = 1;

    status = OK;
exit:
    return status;
    }

/*******************************************************************************
*
* usbSynopsysHcdHostBusUnInit - reset USB Host controller
*
* This routine resets USB Host controller.
*
* RETURNS: N/A
*
*/

LOCAL VOID usbSynopsysHcdHostBusUnInit
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData  /* HCD data */
    )
    {
    /* Reset the Host Controller */

    usbSynopsysHCDResetCore(pHCDData);

    return;
    }
