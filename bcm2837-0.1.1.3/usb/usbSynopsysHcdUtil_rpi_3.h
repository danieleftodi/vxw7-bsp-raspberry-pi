/* usbSynopsysHcdUtil_rpi_3.h - utility Functions for Synopsys HCD */

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

This contains the prototypes of the utility functions
which are used by the Synopsys USB host controller Driver.

*/

#ifndef __INCSynopsysHcdUtil_rpi_3h
#define __INCSynopsysHcdUtil_rpi_3h

#include <ffsLib.h>

#ifdef	__cplusplus
extern "C" {
#endif

/* This macro saves the current transfer toggle */

#define USB_SYNOPSYSHCD_SAVE_DATA_TOGGLE(pHCDData,pHCDPipe,uChannel)          \
    {                                                                         \
    if (0 == (USB_SYNOPSYSHCD_HCTSIZ_PID &                                    \
        USB_SYNOPSYSHCD_READ32_REG(pHCDData,USB_SYNOPSYSHCD_HCTSIZ(uChannel)) )) \
        pHCDPipe->uPidToggle = 0;                                             \
    else                                                                      \
        pHCDPipe->uPidToggle = 1;                                             \
    }                                                                         \

/* This macro gets current transfer toggle */

#define USB_SYNOPSYSHCD_GET_DATA_TOGGLE(pHCDPipe)                             \
        ((pHCDPipe->uPidToggle) ? 2 : 0)

/* This macro returns the full frame number according the micro frame number */

#define USB_SYNOPSYSHCD_GET_FULL_FRAME_NUMBER(uMicroFrameNum)                  \
        ((uMicroFrameNum >> 3) % USB_SYNOPSYSHCD_FULL_FRAME_NUM_MAX)

/* This macro judges the pipe is periodic or not */

#define USB_SYNOPSYSHCD_IS_PERIODIC(pHCDPipe)                                 \
        ((pHCDPipe->uEndpointType == USB_ATTR_INTERRUPT) ||                   \
         (pHCDPipe->uEndpointType == USB_ATTR_ISOCH))

/* This macro judges the pipe need split or not */

#define USB_SYNOPSYSHCD_SHOULD_SPLIT(pHCDData,pHCDPipe)                       \
    (((pHCDPipe->uSpeed != USBHST_HIGH_SPEED) &&                              \
      (USB_SYNOPSYSHCD_HPRT_PRTSPD_HIGH ==                                    \
       (USB_SYNOPSYSHCD_READ32_REG(pHCDData,USB_SYNOPSYSHCD_HPRT) &           \
        USB_SYNOPSYSHCD_HPRT_PRTSPD))) ? TRUE : FALSE)                           \

/* Get the first idle dma channel from the channel map */

#define USB_SYNOPSYSHCD_GET_FIRST_IDLE_DMACHANNEL(uIdleDmaChannelMap)         \
    (ffsLsb(uIdleDmaChannelMap)-1)

/* This macro indicates to find the first set bit */

#define USB_SYNOPSYSHCD_FIND_FIRST_SET_BIT(uValue)                            \
    (ffsLsb(uValue)-1)

#define USB_SYNOPSYSHCD_PIPE_HALT_VALUE    (10)

UINT32 usbSynopsysGetTransferLength
    (
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo,
    UINT32                        uUsbHctSizeReg,
    UINT32                        uUsbHccharReg
    );
VOID usbSynopysHcdDeleteRequestFromScheduleList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo
    );
VOID usbSynopysHcdDeleteRequestFromReadyList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo
    );
VOID usbSynopsysHcdAddRequestIntoSendList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo
    );
STATUS usbSynopsysHcdMoveRequetToReadyList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo
    );
STATUS usbSynopsysHcdMoveRequetToSendList
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO pRequestInfo
    );
void usbSynopsysHcdUrbComplete
    (
    pUSBHST_URB            pUrb,
    int                    status
    );
void usbSynopsysHcdCleanPipe
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,    
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe
    );
void usbSynopsysHcdHaltChannel
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,   
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe    
    );
void usbSynopsysHcdUnHaltChannel
    (
    pUSB_SYNOPSYSHCD_DATA         pHCDData,   
    pUSB_SYNOPSYSHCD_PIPE         pHCDPipe    
    );
pUSB_SYNOPSYSHCD_REQUEST_INFO usbSynopsysHcdCreateReq
    (
    pUSB_SYNOPSYSHCD_PIPE pHCDPipe
    );
void usbSynopsysHcdDeleteReq
    (
    pUSB_SYNOPSYSHCD_PIPE          pHCDPipe,
    pUSB_SYNOPSYSHCD_REQUEST_INFO  pRequest
    );
pUSB_SYNOPSYSHCD_REQUEST_INFO usbSynopsysHcdFirstReqGet
    (
    pUSB_SYNOPSYSHCD_PIPE          pHCDPipe
    );
pUSB_SYNOPSYSHCD_PIPE usbSynopsysHcdNewPipe(void);
void usbSynopsysHcdDestroyPipe
    (
    pUSB_SYNOPSYSHCD_PIPE pHCDPipe
    );

extern STATUS vxbUsbSynopsysHciUnRegister (void);

/* define some forward refs for vxbUsbSynopsysHcd.c  */

extern STATUS usbSynopsysHcdDataInitialize(pUSB_SYNOPSYSHCD_DATA);

extern STATUS usbSynopsysHcdDataUninitialize(pUSB_SYNOPSYSHCD_DATA);

#ifdef	__cplusplus
}
#endif

#endif /* __INCSynopsysHcdUtil_rpi_3h*/

/* End of file */
