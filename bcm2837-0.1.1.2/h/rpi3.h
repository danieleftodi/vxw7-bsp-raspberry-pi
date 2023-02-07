/* rpi3.h - Raspberry Pi 3 support header */

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
08mar19,npc  created (F11409)
*/

#ifndef __INCrpi3h
#define __INCrpi3h

#ifdef __cplusplus
extern "C" {
#endif

/* handy sizes */

#define SZ_1K                           (0x00000400)
#define SZ_4K                           (0x00001000)
#define SZ_8K                           (0x00002000)
#define SZ_16K                          (0x00004000)
#define SZ_32K                          (0x00008000)
#define SZ_64K                          (0x00010000)
#define SZ_128K                         (0x00020000)
#define SZ_256K                         (0x00040000)
#define SZ_512K                         (0x00080000)
#define SZ_1M                           (0x00100000)
#define SZ_2M                           (0x00200000)
#define SZ_4M                           (0x00400000)
#define SZ_8M                           (0x00800000)
#define SZ_16M                          (0x01000000)
#define SZ_32M                          (0x02000000)
#define SZ_64M                          (0x04000000)
#define SZ_128M                         (0x08000000)
#define SZ_256M                         (0x10000000)
#define SZ_512M                         (0x20000000)
#define SZ_1G                           (0x40000000)
#define SZ_2G                           (0x80000000)

#define RPI3_READ_32(reg)               (*(volatile UINT32 *)(reg))
#define RPI3_WRITE_32(reg, data)        (*((volatile UINT32 *)(reg)) = (data))
#define RPI3_SETBIT_32(addr, val) \
                       RPI3_WRITE_32(addr, RPI3_READ_32(addr) | (val))
#define RPI3_CLRBIT_32(addr, val) \
                       RPI3_WRITE_32(addr, RPI3_READ_32(addr) & ~(val))
#define RPI3_READ_16(reg)               (*(volatile UINT16 *)(reg))
#define RPI3_WRITE_16(reg, data)        (*((volatile UINT16 *)(reg)) = (data))
#define RPI3_READ_8(reg)                (*(volatile UINT8 *)(reg))
#define RPI3_WRITE_8(reg, data)         (*((volatile UINT8 *)(reg)) = (data))

/* UART */

#define UART_REG_LSR_BIT_TEMT           (1U << 6)
#define UART_REG_LSR                    (0x14)
#define UART_REG_THR                    (0x0)

/* function declarations */

IMPORT  BOOL   rpi3Probe (char * compat);
IMPORT  void   rpi3Init (void);
IMPORT  void   rpi3EarlyInit (void);
IMPORT  char * rpi3Model (void);
IMPORT  void   rpi3UsDelay (int us);
IMPORT  void   rpi3Reset (int startType);
#ifdef _WRS_CONFIG_SMP
IMPORT  UINT32 rpi3CpuAvail (void);
IMPORT  STATUS rpi3CpuEnable (UINT32 cpuId, WIND_CPU_STATE * cpuState);
#endif /* _WRS_CONFIG_SMP */

#ifdef __cplusplus
}
#endif

#endif /* __INCrpi3h */
