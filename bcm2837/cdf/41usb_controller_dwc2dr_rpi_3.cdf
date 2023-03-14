/* 41usb_controller_dwc2dr_rpi_3.cdf - USB controller dwc2dr component description file */

/*
 * Copyright (c) 2019 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
modification history
--------------------
20may19,hkl  created (F11409)
*/

/*
DESCRIPTION
This file contains descriptions for the USB controller dwc2dr components.
*/

/* dwc2dr configuration */

Folder    FOLDER_USB_SYNOPSYSHCI_RPI_3 {
    NAME            Raspberry Pi 3 USB Synopsys HCI Controller
    SYNOPSIS        Raspberry Pi 3 USB Synopsys HCI Controller Components
    _CHILDREN       FOLDER_USB_CONTROLLER
}

Component INCLUDE_SYNOPSYSHCI_RPI_3 {
    NAME            Raspberry Pi 3 Synopsys HCI
    SYNOPSIS        Raspberry Pi 3 Synopsys Host Controller Interface Instance
    _CHILDREN       FOLDER_USB_SYNOPSYSHCI_RPI_3
    MODULES         usbSynopsysHcdInitExit_rpi_3.o
    REQUIRES        INCLUDE_USB    \
                    INCLUDE_USBD
    LINK_SYMS       usbSynopsysHcdRpi3Init \
                    vxbFdtUsbSynopsysHcdRpi3Drv
    PROTOTYPE       STATUS usbSynopsysHcdRpi3Init (void);
    INIT_RTN        usbSynopsysHcdRpi3Init ();
    _INIT_ORDER     usrUsbPreStageInit
    INIT_AFTER      usbInit
}

Component INCLUDE_SYNOPSYSHCI_INIT_RPI_3 {
    NAME            Raspberry Pi 3 Synopsys HCI Init
    SYNOPSIS        Initializes Raspberry Pi 3 Synopsys USB HOST Controller Driver
    _CHILDREN       FOLDER_USB_SYNOPSYSHCI_RPI_3
    REQUIRES        INCLUDE_USB_INIT    \
//                    INCLUDE_HCD_BUS     \
                    INCLUDE_SYNOPSYSHCI_RPI_3
}
