/* vxbUsbSynopsysHcd_rpi_3.h - VxBus Gen2 USB SYNOPSYSHCD Driver Header */

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

#ifndef __vxbUsbSynopsysHcd_rpi_3h
#define __vxbUsbSynopsysHcd_rpi_3h

/* includes */

#include <usbOsal.h>
#include <usbHst.h>
#include "usbSynopsysHcdDataStructures_rpi_3.h"
#include "usbSynopsysHcdEventHandler_rpi_3.h"
#include "usbSynopsysHcdInterfaces_rpi_3.h"
#include "usbSynopsysHcdHardwareAccess_rpi_3.h"
#include "usbSynopsysHcdRegisterInfo_rpi_3.h"
#include "usbSynopsysHcdUtil_rpi_3.h"

#include <usbHcdInstr.h>
#include <spinLockLib.h>
#include <string.h>
#include <rebootLib.h>
#include <hookLib.h>
#include <hwif/vxBus.h>
#include <vxbUsbPhyLib.h>
#include <hwif/buslib/vxbPciLib.h>
#include <hwif/util/vxbDmaBufLib.h>
#ifdef _WRS_CONFIG_FDT
#include <hwif/buslib/vxbFdtLib.h>
#endif /* _WRS_CONFIG_FDT */

#include <pmapLib.h>
#include <vmLibCommon.h>

#ifdef    __cplusplus
extern "C" {
#endif

/* globals */

/* To hold the array of pointers to the HCD maintained data structures */

extern pUSB_SYNOPSYSHCD_DATA * g_pSynopsysHCDData;

/* To hold the handle returned by the USBD during HC driver registration */

extern UINT32  g_SynopsysHCDHandle;

/* Number of host controllers present in the system */

extern UINT32  g_SynopsysHCDControllerCount;

/* Event used for synchronising the access of the free lists */

extern OS_EVENT_ID g_SynopsysHcdListAccessEvent;

/* Function to destroy the periodic interrupt tree */

/* locals */

#ifdef USB_SHCD_POLLING_MODE

/* To store the Polling thread ID */

extern OS_THREAD_ID    synopsysHcdPollingTh[USB_MAX_SYNOPSYSHCI_COUNT] ;

#endif

/*
 * Array of Spinlocks for the SYNOPSYSHCD controllers.  The array will be
 * allocated in the usbSynopsysHcdInit once the maxSynopsysHcdCount is known.
 */

extern spinlockIsr_t spinLockIsrSynopsysHcd[USB_MAX_SYNOPSYSHCI_COUNT];

typedef struct usb_synopsyshcd_cfg
    {
    STATUS (* pPerInitHook)(void *);   /* The hook be called before init */
    STATUS (* pPostResetHook)(void *); /* The hook be called after reset */
    STATUS (* pUnInitHook)(void *);    /* The hook be called after uninit */
    UINT32 (* pDescSwap)(UINT32);      /* Function for HC data structure swap */
    UINT32 (* pUsbSwap)(UINT32);       /* Function for USB endian swap */
    UINT32 (* pRegSwap)(UINT32);       /* Function for HC register endian swap */
    ULONG   uPhyBaseAddr;              /* The Phy based address */  
    UINT32  uPlatformType;             /* Platform type */
    UINT32  uRegOffset;                /* Base reg offset */
    BOOLEAN bFixupPortNumber;          /* Need fix up the port number */
    BOOLEAN bDescBigEndian;            /* The describe reg is big endian */
    BOOLEAN bRegBigEndian;             /* Registers are big endian */
    BOOLEAN bIntEachTD;                /* Need int with each TD */
    BOOLEAN bHasCompanion;             /* Has companion or not */
    VXB_BUSTYPE_ID uDevClass;          /* Controller class, FDT or PCI/PCIe */
    void *  pPhyDev;                   /* USB PHY Dev pointer */
    USB_PHY_TYPE phyType;              /* Phy type */
    void *  pExtData;                  /* Ext Data */

	UINT8	addressMode64;		 /* Indicate the HC's address
											  * mode is 64 bit or not
											  */
    UINT8 	usbNumPorts;		   /* Count of usb ports */
    UINT8 	hostNumDmaChannels;  /* Count of host channel */
    UINT8 	usbDevNum;                   /* Device ID: USB0 or USB1 */
	
	pVOID	(*pBusToCpu)(UINT32 addr); /* Function pointer to hold the
											 * function doing physical address
											 * to CPU Address Conversion
											 */
	
	UINT32	(*pCpuToBus)(pVOID pAddr); /* Function pointer to hold the
											 * function doing CPU Memory to
											 * physical memory conversions
											 */
    }USB_SYNOPSYSHCD_CFG, *pUSB_SYNOPSYSHCD_CFG;

typedef enum
    {
    USB_SYNOPSYSOTG_MODE_NONE,
    USB_SYNOPSYSOTG_MODE_HOST,
    USB_SYNOPSYSOTG_MODE_DEVICE
    } USB_SYNOPSYSOTG_MODE;

#ifdef    __cplusplus
}
#endif

#endif /* __vxbUsbSynopsysHcd_rpi_3h */

