/* usbSynopsysHcdInterfaces_rpi_3.h - interfaces registered with USBD */

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

This contains the interfaces which are registered with
the USBD.

*/

#ifndef __INCSynopsysHcdInterfaces_rpi_3h
#define __INCSynopsysHcdInterfaces_rpi_3h

#ifdef	__cplusplus
extern "C" {
#endif

IMPORT USBHST_STATUS  usbSynopsysHcdCreatePipe
    (
    UINT8     uBusIndex,
    UINT8     uDeviceAddress,
    UINT8     uDeviceSpeed,
    UCHAR   * pEndpointDescriptor,
    UINT16    uHighSpeedHubInfo,
    ULONG   * puPipeHandle
    );

IMPORT USBHST_STATUS  usbSynopsysHcdDeletePipe
    (
    UINT8   uBusIndex,
    ULONG   uPipeHandle
    );

IMPORT USBHST_STATUS  usbSynopsysHcdModifyDefaultPipe
    (
    UINT8   uBusIndex,
    ULONG   uDefaultPipeHandle,
    UINT8   uDeviceSpeed,
    UINT8   uMaxPacketSize,
    UINT16  uHighSpeedHubInfo
    );

IMPORT USBHST_STATUS  usbSynopsysHcdIsBandwidthAvailable
    (
    UINT8    uBusIndex,
    UINT8    uDeviceAddress,
    UINT8    uDeviceSpeed,
    UCHAR  * pCurrentDescriptor,
    UCHAR  * pNewDescriptor
    );

IMPORT USBHST_STATUS usbSynopsysHcdSubmitURB
    (
    UINT8       uBusIndex,
    ULONG       uPipeHandle,
    pUSBHST_URB pURB
    );

IMPORT USBHST_STATUS usbSynopsysHcdCancelURB
    (
    UINT8       uBusIndex,
    ULONG       uPipeHandle,
    pUSBHST_URB pURB
    );

IMPORT USBHST_STATUS usbSynopsysHcdPipeControl
    (
    UINT8  uBusIndex,
    ULONG  uPipeHandle,
    USBHST_PIPE_CONTROL_INFO * pPipeCtrl
    );

IMPORT USBHST_STATUS  usbSynopsysHcdIsRequestPending
    (
    UINT8    uBusIndex,
    ULONG    uPipeHandle
    );

IMPORT USBHST_STATUS usbSynopsysHcdGetFrameNumber
    (
    UINT8     uBusIndex,
    UINT16  * puFrameNumber
    );

IMPORT USBHST_STATUS usbSynopsysHcdSetBitRate
    (
    UINT8     uBusIndex,
    BOOL      bIncrement,
    UINT32  * puCurrentFrameWidth
    );


IMPORT USBHST_STATUS   usbSynopsysHcdResetTTRequestComplete
    (
    UINT8         uRelativeBusIndex,
    VOID *        pContext,
    USBHST_STATUS nStatus
    );

IMPORT USBHST_STATUS   usbSynopsysHcdClearTTRequestComplete
    (
    UINT8         uRelativeBusIndex,
    VOID *        pContext,
    USBHST_STATUS nStatus
    );

#ifdef	__cplusplus
}
#endif

#endif /* __INCSynopsysHcdInterfaces_rpi_3h */

/* End of file */
