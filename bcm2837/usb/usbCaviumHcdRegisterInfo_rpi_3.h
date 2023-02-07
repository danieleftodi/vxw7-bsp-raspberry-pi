/* usbCaviumHcdRegisterInfo_rpi_3.h - CAVIUM specific definitions for Synopsys
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

This file contains the register definitions for CAVIUM chips using Synopsys
Dual-Role USB Controller IP.

*/

#ifndef __INCCaviumHcdRegisterInfo_rpi_3h
#define __INCCaviumHcdRegisterInfo_rpi_3h

#ifdef  __cplusplus
extern "C" {
#endif


/* The following register definitions are only specific for CAVIUM chips */

#define USB_CAVIUMHCD_USBN_INT_SUM                    (0x0001180068000000ull)
#define USB_CAVIUMHCD_USBN_INT_ENB                    (0x0001180068000008ull)
#define USB_CAVIUMHCD_USBN_BIST_STATUS                (0x00011800680007F8ull)
#define USB_CAVIUMHCD_USBN_CTL_STATUS                 (0x00016F0000000800ull)
#define USB_CAVIUMHCD_USBN_DMA_TEST                   (0x00016F0000000808ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN0              (0x00016F0000000818ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN1              (0x00016F0000000820ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN2              (0x00016F0000000828ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN3              (0x00016F0000000830ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN4              (0x00016F0000000838ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN5              (0x00016F0000000840ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN6              (0x00016F0000000848ull)
#define USB_CAVIUMHCD_USBN_DMA0_INB_CHN7              (0x00016F0000000850ull)

#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN0             (0x00016F0000000858ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN1             (0x00016F0000000860ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN2             (0x00016F0000000868ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN3             (0x00016F0000000870ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN4             (0x00016F0000000878ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN5             (0x00016F0000000880ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN6             (0x00016F0000000888ull)
#define USB_CAVIUMHCD_USBN_DMA0_OUTB_CHN7             (0x00016F0000000890ull)

#ifdef  __cplusplus
}
#endif

#endif /* __INCCaviumHcdRegisterInfo_rpi_3h */

/* End of file */
