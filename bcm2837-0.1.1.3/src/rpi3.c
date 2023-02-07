/* rpi3.c - Raspberry Pi 3 support library */

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

/*
DESCRIPTION
This library provides board-specific routines for the Raspberry Pi 3.

INCLUDE FILES: vxBus.h boardLib.h pmapLib.h

SEE ALSO:
\tb VxWorks Programmer's Guide: Configuration
\tb "ARMv8-A Architecture Reference Manual"
*/

/* includes */

#include <vxWorks.h>
#include <stdio.h>
#include <boardLib.h>
#include <sysLib.h>
#include <string.h>
#include <vxLib.h>
#include <cacheLib.h>
#include <pmapLib.h>
#include <vmLib.h>
#include <intLib.h>
#include <vxFdtLib.h>
#include <vxFdtCpu.h>
#include <arch/arm/mmuArmLib.h>
#include <private/vmLibP.h>
#include <hwif/vxBus.h>
#include <subsys/timer/vxbTimerLib.h>
#include <private/adrSpaceLibP.h>
#include <rpi3.h>

#ifdef _WRS_CONFIG_SMP
#   include <arch/arm/vxAtomicArchLib.h>
#   include <cpuset.h>
#   include <private/cpcLibP.h>
#endif /* _WRS_CONFIG_SMP */

#include <rpi3.h>

/* defines */

#undef DEBUG_RPI_3
#ifdef DEBUG_RPI_3
#undef LOCAL
#define LOCAL
#include <private/kwriteLibP.h>     /* _func_kprintf */

#undef DEBUG_MSG
#define DEBUG_MSG(...)                                  \
    do                                                  \
        {                                               \
        if (_func_kprintf != NULL)                      \
            {                                           \
            (* _func_kprintf)(__VA_ARGS__);             \
            }                                           \
        }                                               \
    while ((FALSE))
#else
#undef DEBUG_MSG
#define DEBUG_MSG(...)
#endif  /* DEBUG_RPI_3 */

/* define for early debug out */

#undef DEBUG_EARLY_PRINT

#define RPI_3_SIO_COMPATIBLE_STR            "brcm,bcm2835-aux-uart"
#define RPI_3_WDG_COMPATIBLE_STR            "brcm,bcm2837-pm-wdt"

#define RPI_3_WDG_WDG                       0x24u
#define RPI_3_WDG_RSTC                      0x1cu
#define RPI_3_WDG_PASSWORD                  0x5a000000u
#define RPI_3_WDG_RSTC_WRCFG_MASK           0x00000030u
#define RPI_3_WDG_RSTC_WRCFG_FULL_RESET     0x00000020u
#define RPI_3_WDG_WDOG_TIMEOUT              0x0000000au

#define RPI_CPUS_BUF_LEN                    20

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#    define SWAP64 vxbSwap64
#else
#    define SWAP32
#    define SWAP64
#endif /* ARMBE8 */

/* imports */

IMPORT UINT32 vxMpidrGet (void);
IMPORT UINT32 vxCpuIdGetByIndex (UINT32);

IMPORT void armSysToMonitor (FUNCPTR core0ExitFunc, FUNCPTR pspExitFunc,
                             int startType);
IMPORT void sysInit (void);

#ifdef _WRS_CONFIG_SMP
IMPORT STATUS armMonitorSpinRelease (UINT32);
IMPORT STATUS armMpCoreInit (UINT32, WIND_CPU_STATE *, FUNCPTR);
#endif /* _WRS_CONFIG_SMP */

IMPORT VIRT_ADDR mmuPtMemBase;

/* locals */

LOCAL BOOL      gicAddrMapValid;

LOCAL VIRT_ADDR debugSioBase = 0;

/* ARM core generic timer frequency (in Hz) */

LOCAL UINT32    rpi3GenTimerFreq;

LOCAL VIRT_ADDR watchDogAddr = (VIRT_ADDR) PMAP_FAILED;

/* forward declarations */

LOCAL STATUS    rpi3ConsoleGet (PHYS_ADDR * pPhyAddr, size_t * pLen);
LOCAL void      rpi3DebugInit ();
LOCAL STATUS    rpi3Dbg (char * buffer, size_t len);
LOCAL void      rpi3Core0Exit(UINT32 apCore);
LOCAL UINT32    rpi3CounterFreqGet  (void);

#ifdef _WRS_CONFIG_SMP
LOCAL void      rpi3PspEntry (void);
LOCAL void      rpi3PspExit (UINT32 apCore);
#endif /*_WRS_CONFIG_SMP*/

/*******************************************************************************
*
* rpi3Probe - probe the board
*
* This routine probes the board with the DTB.
*
* RETURNS: TRUE when probed, otherwise FALSE.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

BOOL rpi3Probe
    (
    char * boardCompatStr
    )
    {
    int    offset;

    /* The board compatibility string is in the root node */

    offset = vxFdtPathOffset ("/");

    if (offset < 0)
        {
        /* badly formed device tree - can't find root node */

        return FALSE;
        }

    /* Determine if the PSL supports this board */

    return (vxFdtNodeCheckCompatible (offset, boardCompatStr) == 0);
    }

/*******************************************************************************
*
* rpi3ConsoleGet - get the configuration for the debug console
*
* This routine searches the debug console, and finds out the register address
* and length.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL STATUS rpi3ConsoleGet
    (
    PHYS_ADDR * pPhyAddr,
    size_t *    pLen
    )
    {
    int         offset;

    /* find the stdout in chosen node */

    offset = vxFdtStdoutGet ();

    if (offset < 0)
        {
        offset = vxFdtNodeOffsetByCompatible (0, RPI_3_SIO_COMPATIBLE_STR);
        if (offset < 0)
            {
            return ERROR;
            }
        }

    return vxFdtDefRegGet (offset, 0, pPhyAddr, pLen);
    }

#ifdef DEBUG_EARLY_PRINT

/*******************************************************************************
*
* rpi3EarlyDebugInit - initialize debug console early
*
* This routine searches the debug console, and set up mmu entries for the
* register space, then initializes the krpintf and kwrite hooks.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void rpi3EarlyDebugInit (void)
    {
    PHYS_ADDR   phyAddr;
    size_t      len;

    if (rpi3ConsoleGet (&phyAddr, &len) != OK)
        {
        DEBUG_MSG ("%s: get sio BAR failed.\n", __FUNCTION__);
        return;
        }

    debugSioBase = vxMmuEarlyRegMap (phyAddr, len);
    _func_kwrite = rpi3Dbg;

    DEBUG_MSG ("Early debug init successfully\n");
    }

#endif /* DEBUG_EARLY_PRINT */

/*******************************************************************************
*
* rpi3EarlyInit - early initialize the board
*
* This routine does early initialization for rpi3 board.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void rpi3EarlyInit (void)
    {
    UINT    uState = MMU_ATTR_VALID | MMU_ATTR_SUP_RWX |
                     MMU_ATTR_CACHE_COPYBACK
#ifdef _WRS_CONFIG_SMP
                     | MMU_ATTR_CACHE_COHERENCY
#endif /* _WRS_CONFIG_SMP */
                     ;

    /* GIC register mapping is not valid */

    gicAddrMapValid = FALSE;

    /* Reset debug UART base virtual address */

    debugSioBase = 0;

    /* Initialise the generic timer frequency */

    rpi3GenTimerFreq = rpi3CounterFreqGet();

#ifdef DEBUG_EARLY_PRINT
    if (boardFlagCheck (BOARD_DESC_FLAG_DBG, FALSE))
        {
        rpi3EarlyDebugInit ();
        }
#endif /* DEBUG_EARLY_PRINT */

    sysPhysMemDescNumEnt = vxFdtPhysMemInfoGet (sysPhysMemDesc,
                                                sysPhysMemDescNumEnt,
                                                uState);

    if (sysPhysMemDescNumEnt == 0)
        {
        DEBUG_MSG ("sysPhysMemDescNumEnt == 0\n");
        }

#ifdef _WRS_CONFIG_LP64
    mmuPtMemBase = KERNEL_SYS_MEM_RGN_BASE - sysPhysMemDesc[0].physicalAddr;
#endif
    }

/*******************************************************************************
*
* rpi3Model - return the model name of the CPU board
*
* This routine returns the model name of the CPU board.
*
* RETURNS: A pointer to a string identifying the board and CPU.
*
* \NOMANUAL
*
* ERRNO: N/A
*/

char * rpi3Model (void)
    {
    int     offset;
    void *  pValue;

    if ((offset = vxFdtPathOffset ("/")) >= 0)
        {
        pValue = (void *) vxFdtPropGet (offset, "model", NULL);
        if (pValue != NULL)
            {
            return (char *) pValue;
            }
        }

    return ("unknown rpi board");
    }

/*******************************************************************************
*
* rpi3Init - initialize the system hardware
*
* This routine initializes various feature of the rpi3 soc. It sets up
* the control registers, initializes various devices if they are present.
*
* NOTE: This routine should not be called directly by the user.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void rpi3Init (void)
    {
    int         offset = -1;
    PHYS_ADDR   phyAddr = 0;
    size_t      len = 0;

    if (boardFlagCheck (BOARD_DESC_FLAG_DBG, FALSE))
        {
        rpi3DebugInit ();
        }

    /* init watchdog base address */

    offset = vxFdtNodeOffsetByCompatible (0, RPI_3_WDG_COMPATIBLE_STR);
    if (offset >= 0)
        {
        if (vxFdtDefRegGet (offset, 0, &phyAddr, &len) == OK)
            {
            watchDogAddr = (VIRT_ADDR) pmapGlobalMap (phyAddr,
                                                      len,
                                                      VXB_REG_MAP_MMU_ATTR);

            if (watchDogAddr == (VIRT_ADDR) PMAP_FAILED)
                {
                DEBUG_MSG ("pmapGlobalMap watchDogAddr failed!\n");
                return;
                }

            DEBUG_MSG ("pmapGlobalMap watchDogAddr success 0x%llx\n",
                       watchDogAddr);
            }
        }

    DEBUG_MSG ("End of BSP init\n");
    }

/*******************************************************************************
*
* rpi3DebugInit - initialize debug console
*
* This routine searches the debug console, and sets up MMU entries for the
* register space, then initializes the krpintf and kwrite hooks.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void rpi3DebugInit (void)
    {
    void *      virtAddr = 0;
    PHYS_ADDR   phyAddr;
    size_t      len;

    if (rpi3ConsoleGet (&phyAddr, &len) != OK)
        {
        DEBUG_MSG ("%s: get fsl,imx-sio BAR failed.\n", __FUNCTION__);
        return;
        }

    virtAddr = pmapGlobalMap (phyAddr, len, VXB_REG_MAP_MMU_ATTR);
    if (virtAddr == PMAP_FAILED)
        {
        return;
        }

    debugSioBase = (VIRT_ADDR) virtAddr;
    _func_kwrite = rpi3Dbg;

    DEBUG_MSG ("Debug init successfully\n");
    }

/*******************************************************************************
*
* rpi3Dbg - output buffer on debug console
*
* This routine outputs the content of the buffer on the debug console.
*
* RETURNS: OK, or ERROR if the parameter is wrong.
*
* ERRNO: N/A
*/

LOCAL STATUS rpi3Dbg
    (
    char *      buffer,
    size_t      len
    )
    {
    if (debugSioBase == (VIRT_ADDR) PMAP_FAILED)
        {
        return ERROR;
        }

    if ((buffer == NULL) || (len == 0))
        {
        return OK;
        }

    while (len-- > 0)
        {
        if (*buffer == '\n')
            {
            while ((RPI3_READ_32 (debugSioBase + UART_REG_LSR)
                    & UART_REG_LSR_BIT_TEMT) == 0)
                {
                }
            RPI3_WRITE_32 (debugSioBase + UART_REG_THR, '\r');
            }

        while ((RPI3_READ_32 (debugSioBase + UART_REG_LSR)
                & UART_REG_LSR_BIT_TEMT) == 0)
            {
            }
        RPI3_WRITE_32 (debugSioBase + UART_REG_THR, *buffer);

        buffer++;
        }

    return OK;
    }

/*******************************************************************************
*
* __inline__rpi3GetCntFreq - get the ARM core counter frequency
*
* This routine gets the ARM core counter frequency from the CNTFRQ system
* register.
*
* RETURNS: the ARM core counter frequency
*/

UINT32 __inline__rpi3GetCntFreq (void)
    {
    UINT64  tempVal;
    __asm__ __volatile__ ("MRS %0, CNTFRQ_EL0"
                          : "=r" (tempVal)
                          : );
    return (UINT32) tempVal;
    }

/*******************************************************************************
*
* rpi3CounterFreqGet - get free-running counter frequency
*
* This function gets the ARM core generic timer counter frequency.
*
* RETURNS: counter frequency (in Hz)
*/

LOCAL UINT32 rpi3CounterFreqGet (void)
    {
    return __inline__rpi3GetCntFreq ();
    }

/*******************************************************************************
*
* __inline__rpi3GetVirtTimerCnt - get the ARM core counter value
*
* This routine gets the ARM core virtual counter frequency from the
* CNTVCT system register.
*
* RETURNS: the ARM core counter frequency
*/

UINT64 __inline__rpi3GetVirtTimerCnt (void)
    {
    UINT64  tempVal;
    __asm__ __volatile__ ("MRS %0, CNTVCT_EL0"
                          : "=r" (tempVal)
                          : );
    return tempVal;
    }

/*******************************************************************************
*
* rpi3CounterValueGet - get free-running counter value
*
* This function gets the ARM core generic timer virtual counter value.
*
* The virtual counter rather than the physical counter is used because the
* the virtual counter is always accessible to software running at non-secure
* EL0. Access to the physical counter depends on the configuration of
* system register CNTHCTL so access is not guaranteed.
*
* RETURNS: Counter value
*/

LOCAL UINT64 rpi3CounterValueGet (void)
    {
    /*
     * Reads from CNTVCT can occur speculatively and out of order relative
     * to other instructions. The ISB ensures that the CNTVCT is read after
     * preceding instructions.
     */

    WRS_ASM ("ISB");

    return __inline__rpi3GetVirtTimerCnt();
    }

/*******************************************************************************
*
* rpi3UsDelay - delay (busy-wait) for a number of microseconds
*
* This function delays for the requested number of microseconds.
*
* NOTE: This function performs a busy-wait and does not relinquish the CPU.
*
* RETURNS: N/A
*/

void rpi3UsDelay
    (
    int delayUs   /* microseconds */
    )
    {
    volatile UINT64 oldTicks;
    volatile UINT64 newTicks;
    volatile UINT64 delayTicks;
    volatile UINT64 elapsedTicks = 0;

    /* Confirm delay is non-zero number of microseconds */

    if (delayUs <= 0)
        {
        return;
        }

    /* Convert delay period to counter ticks */

    delayTicks = ((rpi3GenTimerFreq * (UINT64) delayUs) + (1000000 - 1)) /
                 1000000;

    /*
     * Repeatedly read the counter until the elapsed number
     * of ticks is greater than the delay period.
     */

    oldTicks = rpi3CounterValueGet ();

    while (elapsedTicks <= delayTicks)
        {
        newTicks = rpi3CounterValueGet ();

        if (newTicks >= oldTicks)
            {
            elapsedTicks += newTicks - oldTicks;
            }
        else
            {
            elapsedTicks += 0xffffffffffffffffUL -
                            oldTicks + newTicks + 1;
            }

        oldTicks = newTicks;
        }
    }

/*******************************************************************************
*
* rpi3Reset - system reset
*
* This routine resets the board. It is usually called only by reboot() -- which
* services ^X -- and aborts at interrupt level.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void rpi3Reset
    (
    int     startType
    )
    {
    UINT32  rstc;

    sysClkDisable ();

    if ((sysWarmBootFunc == NULL) || (startType == BOOT_CLEAR))
        {
        goto cold;
        }

#ifdef _WRS_CONFIG_SMP
    armSysToMonitor ((FUNCPTR) rpi3Core0Exit, (FUNCPTR) rpi3PspExit,
                     startType);
#else
    armSysToMonitor ((FUNCPTR) rpi3Core0Exit, NULL, startType);
#endif

cold:

    DEBUG_MSG ("Try cold boot\n");

    /* now reset */

    if (watchDogAddr != (VIRT_ADDR) PMAP_FAILED)
        {
        rstc = RPI3_READ_32 (watchDogAddr + RPI_3_WDG_RSTC);
        rstc &= ~RPI_3_WDG_RSTC_WRCFG_MASK;
        rstc |= RPI_3_WDG_RSTC_WRCFG_FULL_RESET;
        RPI3_WRITE_32 (watchDogAddr + RPI_3_WDG_WDG,
                       RPI_3_WDG_PASSWORD | RPI_3_WDG_WDOG_TIMEOUT);
        RPI3_WRITE_32 (watchDogAddr + RPI_3_WDG_RSTC,
                       RPI_3_WDG_PASSWORD | rstc);
        }

    /* we should never fall through here */

    DEBUG_MSG ("rpi3Reset failed, startType=0x%x.\n", startType);
    while (1) {};
    }

/*******************************************************************************
*
* rpi3Core0Exit - core0's shutdown routine
*
* This routine disables the external L2 and GIC.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void rpi3Core0Exit
    (
    UINT32 apCore
    )
    {
    }

#ifdef _WRS_CONFIG_SMP

/*******************************************************************************
*
* rpi3PspEntry - secondary core entry callback function
*
* This routine is the callback function called before secondary cores enter
* kernel. Here is a good place to set the interrupt control to enable secondary
* cores to receive IPI from core 0.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void rpi3PspEntry (void)
    {
    }

/*******************************************************************************
*
* rpi3PspExit - secondary cores exit function
*
* This routine is the exit function for secondary cores. It deactivates the
* active interrupt, clears all remaining pending interrupt and then disables the
* interrupt controller.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void rpi3PspExit
    (
    UINT32 cpuId
    )
    {
    }

/*******************************************************************************
*
* rpi3CpuEnable - enable a multi core CPU
*
* This routine brings a multi core CPU out of reset
*
* RETURNS: OK or ERROR
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS rpi3CpuEnable
    (
    UINT32              cpuId,
    WIND_CPU_STATE *    cpuState
    )
    {
    STATUS              stat;
    char                buf[RPI_CPUS_BUF_LEN] = {0};
    int                 offset;
    const void *        pValue;
    PHYS_ADDR           rlAddr = 0;
    VIRT_ADDR           rlVirtAddr = 0;
    PHYS_ADDR           physAddr;
    size_t              len;

    if (cpuId >= vxFdtCpuAvail ())
        {
        return ERROR;
        }

    if (ERROR == armMpCoreInit (cpuId, cpuState, (FUNCPTR) rpi3PspEntry))
        {
        return ERROR;
        }

    if (sysStartType == BOOT_COLD)
        {
        snprintf (buf, RPI_CPUS_BUF_LEN, "/cpus/cpu@%d", cpuId);
        offset = vxFdtPathOffset (buf);
        if (offset < 0)
            {
            DEBUG_MSG ("rpi3CpuEnable: dts cpus format error\n");
            return ERROR;
            }

        pValue = vxFdtPropGet (offset, "enable-method", NULL);
        if (NULL == pValue)
            {
            DEBUG_MSG ("rpi3CpuEnable: dts no enable-method property\n");
            return ERROR;
            }

        if (0 != strncmp ("spin-table", (char *)pValue, strlen ("spin-table")))
            {
            DEBUG_MSG ("rpi3CpuEnable: enable-method is not spin-table\n");
            return ERROR;
            }

        pValue = vxFdtPropGet (offset, "cpu-release-addr", NULL);
        if (NULL == pValue)
            {
            DEBUG_MSG ("rpi3CpuEnable: dts no cpu-release-addr property\n");
            return ERROR;
            }

        rlAddr = (PHYS_ADDR) vxFdt64ToCpu (*(UINT64*) pValue);
        if (rlAddr == 0)
            {
            DEBUG_MSG ("rpi3CpuEnable: cpu-release-addr is zero\n");
            return ERROR;
            }

        len = sizeof (rlAddr);
        rlVirtAddr = (VIRT_ADDR) pmapGlobalMap (rlAddr,
                                                len,
                                                VXB_REG_MAP_MMU_ATTR);
        if (rlVirtAddr == (VIRT_ADDR) PMAP_FAILED)
            {
            DEBUG_MSG ("rpi3CpuEnable: pmapGlobalMap watchDogAddr failed!\n");
            return ERROR;
            }

        stat = vmTranslate (NULL, (VIRT_ADDR) sysInit, &physAddr);
        if (stat == ERROR)
            {
            DEBUG_MSG ("rpi3CpuEnable: vmTranslate failed for cpuId:%d\n",
                       cpuId);
            return ERROR;
            }

        DEBUG_MSG ("rpi3CpuEnable: rlAddr %p, rlVirtAddr %p, physAddr %p\n",
                   rlAddr, rlVirtAddr, physAddr);

        /*
         * ARMV8 can not map phy 0 address, use workaround
         * It is a known issue V7COR-6340
         */

        *(volatile ULONG *) (0xffffffff80000000UL + rlAddr) = (ULONG) physAddr;
        (void) cacheFlush (DATA_CACHE, (void *) (0xffffffff80000000UL + rlAddr),
                           sizeof (rlAddr));

        WRS_ASM ("dsb sy");
        WRS_ASM ("sev");

        //(void) pmapGlobalUnmap (rlVirtAddr, sizeof (rlAddr));

        return OK;
        }
    else
        {
        return armMonitorSpinRelease (cpuId);
        }
    }
#endif /* _WRS_CONFIG_SMP */
