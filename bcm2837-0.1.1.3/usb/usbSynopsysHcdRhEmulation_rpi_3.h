/* usbSynopsysHcdRhEmulation_rpi_3.h - root hub emulation entries for Synopsys
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

This contains the data structures and prototypes of
functions of the root hub emulation module used by
the Synopsys USB Host Controller Driver.

*/
#ifndef __INCSynopsysHcdRHEmulation_rpi_3h
#define __INCSynopsysHcdRHEmulation_rpi_3h

#ifdef    __cplusplus
extern "C" {
#endif

/* defines */

/* Class specific values */

#define USB_SYNOPSYSHCD_RH_STANDARD_REQUEST         (0x00)
#define USB_SYNOPSYSHCD_RH_CLASS_SPECIFIC_REQUEST   (0x20)
#define USB_SYNOPSYSHCD_RH_REQUEST_TYPE             (0x60)
#define USB_SYNOPSYSHCD_RH_RECIPIENT_MASK           (0x1F)

#define USB_SYNOPSYSHCD_RH_GET_CONFIG_SIZE          (0x01)
#define USB_SYNOPSYSHCD_RH_DESCRIPTOR_BITPOSITION   (0x08)
#define USB_SYNOPSYSHCD_RH_DEVICE_DESC_SIZE         (0x12)
#define USB_SYNOPSYSHCD_RH_CONFIG_DESC_SIZE         (0x19)
#define USB_SYNOPSYSHCD_RH_GET_STATUS_SIZE          (0x02)

/* Add it for Synopsys HCD */

#define USB_SYNOPSYSHCD_RH_MASK_VALUE               (0x02)


IMPORT USBHST_STATUS usbSynopsysHcdRhCreatePipe
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    UINT8                 uDeviceAddress,
    UINT8                 uDeviceSpeed,
    UCHAR *               pEndpointDescriptor,
    ULONG *               puPipeHandle
    );

IMPORT USBHST_STATUS usbSynopsysHcdRHDeletePipe
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    ULONG                 uPipeHandle
    );

IMPORT USBHST_STATUS usbSynopsysHcdRHSubmitURB
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    ULONG                 uPipeHandle,
    pUSBHST_URB           pURB
    );

IMPORT USBHST_STATUS usbSynopsysHcdRHCancelURB
    (
    pUSB_SYNOPSYSHCD_DATA pHCDData,
    ULONG                 uPipeHandle,
    pUSBHST_URB           pURB
    );

#ifdef    __cplusplus
}
#endif

#endif /* __INCSynopsysHcdRHEmulation_rpi_3h */

/* End of file */
