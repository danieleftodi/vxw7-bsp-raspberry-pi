/* sysLib.c - Raspberry Pi 3 Family system-dependent library */

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
08mar19,hkl  created (F11409)
*/

/*
DESCRIPTION
This library provides board-specific routines for Raspberry Pi 3 series
processors.

INCLUDE FILES:

SEE ALSO:
\tb VxWorks Programmer's Guide: Configuration
*/

/* includes */

#include <vxWorks.h>
#include <boardLib.h>
#include <rpi3.h>
#include <prjParams.h>
#include <vxFdtCpu.h>

#ifdef INCLUDE_SHOW_ROUTINES
#include <stdio.h>
#include <stdlib.h>
#include <ioLib.h>
#include <vmLibCommon.h>
#include <pmapLib.h>
#include <vxFdtLib.h>
#include <hwif/vxBus.h>

/* defines */

/* locals */

LOCAL void boardInfo (void);

/* imports */

IMPORT void cpuArmVerShow (void);

#endif /* INCLUDE_SHOW_ROUTINES */

/* locals */

LOCAL BOARD_FUNC_TBL rpi3BoardFuncTbl = {
    /* .earlyInit  = */ rpi3EarlyInit,
    /* .init       = */ rpi3Init,
    /* .reset      = */ rpi3Reset,
    /* .model      = */ rpi3Model,
    /* .usDelay    = */ rpi3UsDelay,
#ifdef _WRS_CONFIG_SMP
    /* .cpuEn      = */ rpi3CpuEnable,
    /* .cpuAvail   = */ vxFdtCpuAvail,
    /* .cpuDis     = */ NULL,
#endif /*_WRS_CONFIG_SMP*/
#ifdef INCLUDE_SHOW_ROUTINES
    /* .infoShow   = */ boardInfo,
#else
    /* .infoShow   = */ NULL,
#endif /* INCLUDE_SHOW_ROUTINES */
    /* .endMacGet  = */ NULL
};

LOCAL BOARD_DESC rpi_3 =
    {
    /* .uVer     = */ BOARD_DESC_VER_3_0,
    /* .pCompat  = */ "raspberrypi,rpi-3",
#if defined(INCLUDE_DEBUG_KPRINTF) || defined(INCLUDE_DEBUG_KPUTS)
    /* .uFlag    = */ BOARD_DESC_FLAG (BOARD_DESC_FLAG_DBG, 0),
#else
    /* .uFlag    = */ BOARD_DESC_FLAG (0, 0),
#endif
    /* .probe    = */ rpi3Probe,
    /* .pFuncTbl = */ &rpi3BoardFuncTbl
    };

BOARD_DEF (rpi_3)

#ifdef INCLUDE_SHOW_ROUTINES

/*******************************************************************************
*
* socInfoShow - print SOC information
*
* This routine prints SOC information.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void socInfoShow (void)
    {
    }

/*******************************************************************************
*
* boardInfo - print board information
*
* This routine prints board information.
*
* RETURNS: N/A
* ERRNO: N/A
*
*/

LOCAL void boardInfo (void)
    {
    printf ("%s, ", rpi3Model ());
    printf ("CPU: ");
    socInfoShow ();
    printf (", ");
    cpuArmVerShow ();
    printf ("\n");

    return;
    }
#endif /* INCLUDE_SHOW_ROUTINES */
