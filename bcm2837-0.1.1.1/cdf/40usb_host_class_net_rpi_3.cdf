/* 40usb_host_class_net_rpi_3.cdf - USB host class network component description file */

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
This file contains descriptions for the USB host class network components.
*/

/* USB host class network configuration */

Component INCLUDE_USB_GEN2_LAN78XX_RPI_3 {
    NAME            Raspberry Pi 3 Lan78xx Serial Controllers
    SYNOPSIS        Raspberry Pi 3 USB Generation 2 Lan78xx serial controllers
    _CHILDREN       FOLDER_USB_GEN2_END_CONTROLLERS
    MODULES         usb2Lan78xx_rpi_3.o usb2End.o
    REQUIRES        INCLUDE_USB \
                    INCLUDE_USB_GEN2_HELPER
    PROTOTYPE       STATUS usb2Lan78xxInit (char * pName);
    INIT_RTN        usb2Lan78xxInit (NULL);
    _INIT_ORDER     usrUsbGroupInit
    INIT_AFTER      usbHstClassInit
}

Component INCLUDE_USB_GEN2_LAN9514_RPI_3 {
    NAME            LAN9514 Ethernet Controller
    SYNOPSIS        Raspberry Pi 3B LAN9514 USB 2.0 Ethernet Controller
    _CHILDREN       FOLDER_USB_GEN2_END_CONTROLLERS
    MODULES         usb2Lan9514_rpi_3.o usb2End.o
    REQUIRES        INCLUDE_USB \
                    INCLUDE_USB_GEN2_HELPER
    PROTOTYPE       STATUS usb2Lan9514Init (char *pName);
    INIT_RTN        usb2Lan9514Init (NULL);
    _INIT_ORDER     usrUsbGroupInit
    INIT_AFTER      usbHstClassInit
}
