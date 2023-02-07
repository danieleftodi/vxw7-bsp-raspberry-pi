/* sysALib.s - Raspberry Pi 3 Family system-dependent routines */

/*
 * Copyright (c) 2019-2020 Wind River Systems, Inc.
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
25jun20,eli  Remove __DCC__ #ifdefs (V7COR-7450)
08mar19,hkl  created (F11409)
*/

/*
DESCRIPTION
This module contains system-dependent routines written in assembly
language. It contains the entry code, sysInit(), for VxWorks images
that start running from RAM, such as 'vxWorks'.  These images are
loaded into memory by some external program (e.g., a boot ROM) and then
started.  The routine sysInit() must come first in the text segment.
Its job is to perform the minimal setup needed to call the generic C
routine usrInit().

sysInit() masks interrupts in the processor and the interrupt
controller and sets the initial stack pointer.  Other hardware and
device initialisation is performed later in the sysHwInit routine in
sysLib.c.

*/

#define _ASMLANGUAGE
#include <vxWorks.h>
#include <vsbConfig.h>
#include <asm.h>
#include <regs.h>
#include "prjParams.h"

/* defines */

#define CNTHCTL_EL2_EL1PCTEN            (0x1)
#define CNTHCTL_EL2_EL1PCEN             (0x1 << 1)

#define HCR_EL2_RW                      (0x1 << 31)
#define HCR_EL2_HCD                     (0x1 << 29)

#define SCR_EL3_NS                      (0x1)
#define SCR_EL3_RES1                    (0x3 << 4)
#define SCR_EL3_SMD                     (0x1 << 7)
#define SCR_EL3_HCE                     (0x1 << 8)
#define SCR_EL3_RW                      (0x1 << 10)

#define CPTR_EL2_RES1                   (0x33ff)
#define SCTLR_EL2_RES1                  (0x30C50830)

#define SPSR_EL3_M_EL2h                 (0x9)
#define SPSR_EL3_F                      (0x1 << 6)
#define SPSR_EL3_I                      (0x1 << 7)
#define SPSR_EL3_A                      (0x1 << 8)
#define SPSR_EL3_E                      (0x1 << 9)

#define SCTLR_EL1_RES1                  (0x30D00800)

#define SPSR_EL2_M_EL1h                 (0x5)
#define SPSR_EL2_F                      (0x1 << 6)
#define SPSR_EL2_I                      (0x1 << 7)
#define SPSR_EL2_A                      (0x1 << 8)
#define SPSR_EL2_E                      (0x1 << 9)

#define CPACR_EL1_FPEN_NO_INST_TRAPPED  (0x3 << 20)

#define SCR_EL3_DEFAULT         (SCR_EL3_NS  | SCR_EL3_RES1 | SCR_EL3_SMD | \
                                 SCR_EL3_HCE | SCR_EL3_RW)

#define SPSR_EL3_DEFAULT        (SPSR_EL3_E | SPSR_EL3_A | SPSR_EL3_I | \
                                 SPSR_EL3_F | SPSR_EL3_M_EL2h)

#define SPSR_EL2_DEFAULT        (SPSR_EL2_E | SPSR_EL2_A | SPSR_EL2_I | \
                                 SPSR_EL2_F | SPSR_EL2_M_EL1h)

        /* exports */

        FUNC_EXPORT (sysInit)
#ifdef _WRS_CONFIG_LP64
        FUNC_EXPORT (armWarmReboot)
#endif /* _WRS_CONFIG_LP64 */

        /* externals */

        FUNC_IMPORT (usrInit)

#ifdef INCLUDE_STANDALONE_DTB

        .data

#define TO_STRING(exp)                  #exp
#define DTS_ASM_FILE(file)              TO_STRING (file.s)
#include DTS_ASM_FILE (DTS_FILE)

#endif /* INCLUDE_STANDALONE_DTB */

#if defined (__llvm__)
        .section .text.entry, "ax"
#elif defined (__GNUC__)
        .section .text.entry, "ax"
#else
#error "TOOL not supported!"
#endif

#ifndef _WRS_CONFIG_LP64
        .code 32
#endif /* _WRS_CONFIG_LP64 */

/*******************************************************************************
*
* sysInit - start after boot
*
* This routine is the system start-up entry point for VxWorks in RAM, the
* first code executed after booting.  It disables interrupts, sets up
* the stack, and jumps to the C routine usrInit() in usrConfig.c.
*
* The initial stack is set to grow down from the address of sysInit().  This
* stack is used only by usrInit() and is never used again.  Memory for the
* stack must be accounted for when determining the system load address.
*
* NOTE: This routine should not be called by the user.
*
* SYNOPSIS
* \ss
* VOID sysInit
*     (
*     void * pDtb   /@ physical pointer for device-tree in RAM  @/
*     )
* \se
*
* NOTE:
*   1. This routine should not be called by the user;
*   2. <pOf> is unused because bootng from Open Firmware is unsupported now;
*   3. The initial stack is placeed in .data section, and mapped with the entire
*   image vxWorks.
*
* RETURNS: N/A
*
* sysInit ()              /@ THIS IS NOT A CALLABLE ROUTINE @/
*
*/

FUNC_LABEL (sysInit)
#ifdef _WRS_CONFIG_LP64

        /* save X0 */

        MOV     X20, X0

        /* read current exception level */

        MRS	X1, CurrentEL
        CMP     X1, 0xC
        B.EQ    el3_mode
        CMP     X1, 0x8
        B.EQ    el2_mode
        CMP     X1, 0x4
        B.EQ    el1_mode

el3_mode:

        /* EL3 */

        MOV     X1, #(SCR_EL3_DEFAULT)  /* SCR */
        MSR     SCR_EL3, X1
        MSR     CPTR_EL3, XZR           /* CPTR_EL3 */

        MOV     X1, #(SPSR_EL3_DEFAULT)
        MSR     SPSR_EL3, X1            /* SPSR_EL3 */

        MRS     X1, VBAR_EL3
        MSR     VBAR_EL2, X1            /* VBAR_EL2 */

        ADR     X0, el2_mode
        MOV     LR, X0
        MSR     ELR_EL3, LR
        ERET

el2_mode:

        /* EL2 */

        /* initilize MPID/MPIDR registers */

        MRS     X1, MIDR_EL1
        MSR     VPIDR_EL2, X1           /* VPIDR_EL2 */

        MRS     X1, MPIDR_EL1
        MSR     VMPIDR_EL2, X1          /* VMPIDR_EL2 */

        /* initialize Generic Timers */

        MRS     X1, CNTHCTL_EL2
        ORR     X1, X1, #(CNTHCTL_EL2_EL1PCTEN | CNTHCTL_EL2_EL1PCEN)
        MSR     CNTHCTL_EL2, X1         /* CNTHCTL_EL2 */
        MSR     CNTVOFF_EL2, XZR        /* CNTVOFF_EL2 */

        MOV     X1, #(SPSR_EL2_DEFAULT)
        MSR     SPSR_EL2, X1            /* SPSR_EL2 */

        MOV     X1, #(CPTR_EL2_RES1)
        MSR     CPTR_EL2, X1            /* CPTR_EL2 */

        LDR     X1, =SCTLR_EL2_RES1
        MSR     SCTLR_EL2, X1           /* SCTLR_EL2 */

        MRS     X0, SCTLR_EL1
        MOV     X1, #SCTLR_C | SCTLR_I
        AND     X0, X0, X1

        LDR     X1, =SCTLR_EL1_RES1
        ORR     X1, X1, X0
        MSR     SCTLR_EL1, X1           /* SCTLR_EL1 */

        MSR     HSTR_EL2, XZR           /* HSTR_EL2 */
        MOV     X1, #(CPACR_EL1_FPEN_NO_INST_TRAPPED)
        MSR     CPACR_EL1, X1           /* CPACR_EL1 */

        LDR     X1, =(HCR_EL2_RW | HCR_EL2_HCD)
        MSR     HCR_EL2, X1             /* HCR_EL2 */

        MRS     X1, VBAR_EL2
        MSR     VBAR_EL1, X1            /* VBAR_EL1 */

        ADR     X0, el1_mode
        MOV     LR, X0
        MSR     ELR_EL2, LR
        ERET                            /* to EL1 */

el1_mode:

        /* EL1 */

        /* set initial stack pointer so stack grows down from start of code */

        ADR     X1, sysInit
        MOV     SP, X1
        MOV     X29, #0
        LDR     X1, =sysInit
        ADR     X2, sysInit
        SUB     X2, X2, X1

#ifdef _WRS_CONFIG_SMP
        MRS     X1, MPIDR_EL1
        ANDS    X1, X1, #0xf
        BNE     2f
#endif

#ifdef INCLUDE_STANDALONE_DTB
        LDR     X0, =dt_blob_start
#else
        MOV     X0, X20
#endif /* INCLUDE_STANDALONE_DTB */

        LDR     X1, =gpDtbInit
        ADD     X1, X1, X2
        STR     X0, [X1]        /* set start of device tree blob */

2:
        MOV     X0, #2      /* BOOT_COLD */
armWarmReboot:
        MOV     X19, X0
        BL      vxCpuInit
        LDR     x0, =STATIC_MMU_TABLE_BASE
        LDR     x1, =LOCAL_MEM_LOCAL_ADRS
        LDR     x2, =IMA_SIZE
        BL      vxMmuEarlyInit

        /* now call usrInit (startType) */

        MOV     X0, X19
        B       usrInit

#else /* !_WRS_CONFIG_LP64 */

#error "32bit not supported"

#endif /* _WRS_CONFIG_LP64 */

FUNC_END (sysInit)
