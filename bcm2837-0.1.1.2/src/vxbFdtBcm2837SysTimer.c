/* vxbFdtBcm2837SysTimer.c - BCM2837 System Timer Module driver */

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
08mar19,wdy  created (F11409)
*/

/*
DESCRIPTION

This is the VxBus compliant timer driver which implements the functionality of
broadcom BCM2837 platform (System Timer Module) driver. In this platform,
System timer module was designed to support commonly required system and
application software timing functions. The System Timer peripheral provides
four 32-bit timer channels and a single 64-bit free running counter. Each
channel has an output compare register, which is compared against the 32 least
significant bits of the free running counter values.When the two values match,
the system timer peripheral generates a signal to indicate a match for the
appropriate channel. The match signal is then fed into the interrupt controller
The interrupt service routine then reads the output compare register and adds
the appropriate offset for the next timer tick. The free running counter is
driven by the timer clock and stopped whenever the processor is stopped in
debug mode.

To add the driver to the vxWorks image, add the following component.

\cs
    vxprj vip component add DRV_TIMER_FDT_BCM2837_SYSTIMER
\ce

In BCM2837 platform, each STM device should be bound to a device tree node
which requires below properties:

\cs
compatible:      Specify the programming model for the device.
                 It should be set to "brcm,bcm2837-system-timer" and is used
                 by vxbus GEN2 for device driver selection.

reg:             Specify the address of the device's resources within
                 the address space defined by its parent bus.

interrupts:      Specify interrupt vector of the interrupts that are generated
                 by this device.

interrupt-parent This property is available to define an interrupt parent.
                 If it is missing froma device, it's interrupt parent is
                 assumed to be its device tree parent.

clock-frequency  This property is available to define the frequency of
                 clock for system timer module.

\ce

Below is an example for BCM2837 platform:

\cs
    systimer: timer@3f003000
        {
        compatible = "brcm,bcm2837-system-timer";
        reg = <0x0 0x3f003000 0x0 0x1000>;
        interrupt-parent = <&intc>;
        interrupts = <35>;
        clock-frequency = <1000000>;
        };
\ce

INCLUDE FILES: vxBus.h vxbTimerLib.h string.h vxbFdtLib.h
*/

/* includes */

#include <vxWorks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hwif/vxBus.h>
#include <hwif/vxbus/vxbLib.h>
#include <hwif/buslib/vxbFdtLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <subsys/clk/vxbClkLib.h>
#include <subsys/int/vxbIntLib.h>

/* defines */

/* debug macro */

#undef  DEBUG_MSG
#undef  BCM_SYSTIMER_DBG
#ifdef  BCM_SYSTIMER_DBG

/* turning local symbols into global symbols */

#ifdef  LOCAL
#undef  LOCAL
#define LOCAL
#endif

#include <private/kwriteLibP.h>     /* _func_kprintf */
#define BCM_SYSTIMER_DBG_OFF             0x00000000
#define BCM_SYSTIMER_DBG_ISR             0x00000001
#define BCM_SYSTIMER_DBG_ERR             0x00000002
#define BCM_SYSTIMER_DBG_INFO            0x00000004
#define BCM_SYSTIMER_DBG_ALL             0xffffffff

LOCAL UINT32 bcmTimerDbgMask = BCM_SYSTIMER_DBG_ALL;

#define DEBUG_MSG(mask, ...)                                                  \
    do                                                                        \
        {                                                                     \
        if ((bcmTimerDbgMask & (mask)) || ((mask) == BCM_SYSTIMER_DBG_ALL))  \
            {                                                                 \
            if (_func_kprintf != NULL)                                        \
                {                                                             \
                (* _func_kprintf) (__VA_ARGS__);                              \
                }                                                             \
            }                                                                 \
        }                                                                     \
    while (FALSE)
#else
#define DEBUG_MSG(...)
#endif  /* BCM_SYSTIMER_DBG */

/* register access through the vxBus access routines */

#undef TIMERFUNC_TO_TIMERDATA

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#    define SWAP64 vxbSwap64
#else
#    define SWAP32
#    define SWAP64
#endif /* ARMBE8 */


#define BCM_TIMER_REG_READ(pTimer, offset)                                    \
        SWAP32(vxbRead32 (pTimer->pHandle,                                    \
                  (UINT32 *)((UINT8 *)(pTimer->regBase) + offset)))

#define BCM_TIMER_REG_WRITE(pTimer, offset, value)                            \
        vxbWrite32 (pTimer->pHandle,                                          \
                    (UINT32 *)((UINT8 *)(pTimer->regBase) + offset),          \
                    SWAP32(value))

#define BCM_TIMER_REG_SETBIT_4(pTimer, offset, data)                          \
        vxbWrite32 (pTimer->pHandle,                                          \
                    (UINT32 *)((UINT8 *)(pTimer->regBase) + offset),          \
                    SWAP32 ((UINT32)(data) |                                  \
                             BCM_TIMER_REG_READ(pTimer, offset)))

#define BCM_TIMER_REG_CLEARBIT_4(pTimer, offset, data)                        \
        vxbWrite32 (pTimer->pHandle,                                          \
                    (UINT32 *)((UINT8 *)(pTimer->regBase) + offset),          \
                    SWAP32 ((UINT32)(~(data)) &                               \
                             BCM_TIMER_REG_READ(pTimer, offset)))

#define TIMERFUNC_TO_TIMERDATA(pTFunc)                                        \
        (BCM_SYSTIMER_MODULE *) ((ULONG)(pTFunc) -                            \
        OFFSET (BCM_SYSTIMER_MODULE, pTimerFunc))

/* timer defines */

#define BCM_SYSTIMER_MAX_COUNT           0xffffffff
#define BCM_SYSTIMER_DEFAULT_FREQ        1000000U
#define BCM_SYSTIMER_DEFAULT_MIN_FREQ    1
#define BCM_SYSTIMER_DEFAULT_MAX_FREQ    10000U
#define DEFAULT_TICKS_PER_SECOND         60U
#define BCM_SYSTIMER_INIT_VALUE          0x0
#define BCM_SYSTIMER_NAME                "bcm2837-system-timer"
#define BCM_SYSTIMER_CHANNEL_DEFAULT     3

#define BCM_SYSTIMER_CMP_START_DEFAULT   0x0

/* Systmer Timer Register Offsets */

#define BCM_SYSTIMER_CS_OFFSET    0x0u /* System Timer Control/Status */
#define BCM_SYSTIMER_CLO_OFFSET   0x4u /* System Timer Counter Lower 32 bits */
#define BCM_SYSTIMER_CHI_OFFSET   0x8u /* System Timer Counter Higher 32 bits */

/* Systmer Channel Compare Register */

#define BCM_SYSTIMER_CHL_CMP(x)   (0x0cu + (x) * 4)

/* Systmer Control Register bit definitions */

/* 0 = No Timer channels match since last cleared.
   1 = Timer channels match detected. */

#define BCM_SYSTIMER_ENABLE_CHL(x)   (0x1u << (x))

/* typedefs */


typedef struct bcmSysTimerInstance
    {
    struct vxbTimerFunctionality pTimerFunc;  /* VxBus timer interface */
    void                         (* pIsrFunc)(_Vx_usr_arg_t); /* ISR func */
    _Vx_usr_arg_t                arg;         /* ISR argument */
    UINT32                       maxCount;    /* max value of count */
    VXB_RESOURCE *               pIntRes;     /* VxBus Interrupt source */
    /* device ID for system timer module */
    VXB_DEV_ID                   pDev;
    spinlockIsr_t                spinLock;    /* system timer module spinlock */
    void *                       regBase;     /* base address for STM module */
    void *                       pHandle;     /* handle for STM module */
    BOOL                         isEnabled;   /* status flag */
    /* compare value temporary storage */
    UINT32                       cmpStartValue;
    } BCM_SYSTIMER_MODULE; /* bcm system timer module */

/* forward declarations */

LOCAL STATUS fdtBcmSytTimerProbe (VXB_DEV_ID pDev);
LOCAL STATUS fdtBcmSytTimerAttach (VXB_DEV_ID pDev);
LOCAL STATUS bcmSystTimerRelease (void * pCookie);
LOCAL STATUS bcmSystTimerRolloverGet (void * pCookie, UINT32 * pCount);
LOCAL STATUS bcmSystTimerCountGet (void * pCookie, UINT32 * pCount);
LOCAL STATUS bcmSystTimerISRSet (void * pCookie, void (*pFunc)(_Vx_usr_arg_t),
                           _Vx_usr_arg_t arg);
LOCAL STATUS bcmSystTimerDisable (void * pCookie);
LOCAL STATUS bcmSystTimerEnable (void * pCookie, UINT32 maxTimerCount);
LOCAL STATUS bcmSystTimerAllocate (void * pCookie, UINT32 flags);
LOCAL void   bcmSystTimerInt (BCM_SYSTIMER_MODULE * pTimer);

/* locals */

LOCAL UINT32 timerUnitNum = 0;

LOCAL VXB_DRV_METHOD fdtBcmSytTimerMethodList[] =
    {
    /* DEVICE API */

    {VXB_DEVMETHOD_CALL(vxbDevProbe),  fdtBcmSytTimerProbe},
    {VXB_DEVMETHOD_CALL(vxbDevAttach), fdtBcmSytTimerAttach},
    {0, NULL}
    };

LOCAL VXB_FDT_DEV_MATCH_ENTRY fdtBcmSytTimerMatch[] =
    {
    {
    "brcm,bcm2837-system-timer", /* compatible */
    NULL
    },
    {}                           /* empty terminated list */
    };

/* globals */

VXB_DRV vxbFdtBcmSytTimerDrv =
    {
    { NULL } ,
    "bcm2837-system-timer",                            /* Name */
    "Broadcom System Timer FDT driver",                /* Description */
    VXB_BUSID_FDT,                                     /* Class */
    0,                                                 /* Flags */
    0,                                                 /* Reference count */
    &fdtBcmSytTimerMethodList[0]                       /* Method table */
    };

VXB_DRV_DEF(vxbFdtBcmSytTimerDrv)

/*******************************************************************************
*
* fdtBcmSytTimerProbe - probe for device presence at specific address
*
* This routine probes for device presence at specific address.
*
* RETURNS: OK if probe passes and assumed a valid (or compatible) device,
* ERROR otherwise.
*/

LOCAL STATUS fdtBcmSytTimerProbe
    (
    VXB_DEV_ID pDev
    )
    {
    return vxbFdtDevMatch (pDev, &fdtBcmSytTimerMatch[0], NULL);
    }

/*******************************************************************************
*
* fdtBcmSytTimerAttach - attach broadcom 2837 device
*
* This is the BCM2837 system timer initialization routine. This routine get
* system timer information from dts and each module of timer channel register
* to vxbus.
* After registration, STM module will be enabled.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcmSytTimerAttach
    (
    VXB_DEV_ID pDev
    )
    {
    BCM_SYSTIMER_MODULE *          pTimer;
    struct vxbTimerFunctionality * pTFunc;
    VXB_RESOURCE *                 pMemRes = NULL;
    VXB_RESOURCE_ADR *             pResAdr = NULL;
    VXB_FDT_DEV *                  pFdtDev = NULL;
    VXB_CLK_ID                     pClk;
    UINT32                         timerClk;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    if (pDev == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Device ID is NULL.\n");
        return ERROR;
        }

    pFdtDev = (VXB_FDT_DEV *) vxbFdtDevGet (pDev);
    if (pFdtDev == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "FDT is NULL.\n");
        return ERROR;
        }

    if (vxbClkEnableAll (pDev) == ERROR)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Failed to enable clocks\n");
        return ERROR;
        }

    /* get input clock frequency */

    pClk = vxbClkGet (pDev, NULL);
    if (pClk == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Failed to get clock ID.\n");
        (void) vxbClkDisableAll (pDev);
        return ERROR;
        }

    timerClk =(UINT32) vxbClkRateGet (pClk);
    if (timerClk == CLOCK_RATE_INVALID)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR,
                   "Failed to get clock rate. Using default frequency\n");
        timerClk = BCM_SYSTIMER_DEFAULT_FREQ;
        }

    /* allocate the memory for the timer structure */

    pTimer = (BCM_SYSTIMER_MODULE *)
                vxbMemAlloc (sizeof (BCM_SYSTIMER_MODULE));
    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR,
                   "Failed timer module memory allocation.\n");
        goto attachFail;
        }

    /* allocate register memory resource */

    pMemRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);
    if ((pMemRes == NULL) || (pMemRes->pRes == NULL))
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR,
                   "Failed register memory resource allocation\n");
        goto attachFail;
        }

    pResAdr = (VXB_RESOURCE_ADR *) pMemRes->pRes;
    pTimer->regBase = (void *) pResAdr->virtual;
    pTimer->pHandle = pResAdr->pHandle;
    pTimer->pDev = pDev;

    /* allocate systimer module interrupt resource */
    pTimer->pIntRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);

    if (pTimer->pIntRes == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR,
                   "Failed system timer interrupt resource allocation\n");
        goto attachFail;
        }

    pTimer->maxCount = BCM_SYSTIMER_MAX_COUNT;
    pTimer->cmpStartValue = BCM_SYSTIMER_CMP_START_DEFAULT;

    /* locate the system timer functionality data structure */
    pTFunc = &pTimer->pTimerFunc;

    pTFunc->minFrequency = BCM_SYSTIMER_DEFAULT_MIN_FREQ;
    pTFunc->maxFrequency = BCM_SYSTIMER_DEFAULT_MAX_FREQ;

    pTFunc->clkFrequency = timerClk;

    /* store the feature provided by the timer */

    pTFunc->features =  VXB_TIMER_CAN_INTERRUPT |
                        VXB_TIMER_INTERMEDIATE_COUNT |
                        VXB_TIMER_SIZE_32 |
                        VXB_TIMER_CANNOT_MODIFY_ROLLOVER |
                        VXB_TIMER_AUTO_RELOAD;

    /* set a default ticks per second */

    pTFunc->ticksPerSecond = DEFAULT_TICKS_PER_SECOND;
    pTFunc->rolloverPeriod = pTimer->maxCount / pTFunc->clkFrequency;

    /* create timer name */

    (void) snprintf (pTFunc->timerName, MAX_DRV_NAME_LEN, "%s",
                     BCM_SYSTIMER_NAME);
    pTFunc->timerNo = timerUnitNum++;

    pTimer->pIsrFunc = NULL;

    /* populate the function pointer */

    pTFunc->timerAllocate = bcmSystTimerAllocate;
    pTFunc->timerRelease = bcmSystTimerRelease;
    pTFunc->timerRolloverGet = bcmSystTimerRolloverGet;
    pTFunc->timerCountGet = bcmSystTimerCountGet;
    pTFunc->timerDisable = bcmSystTimerDisable;
    pTFunc->timerEnable = bcmSystTimerEnable;
    pTFunc->timerISRSet = bcmSystTimerISRSet;

    pTimer->isEnabled = FALSE;

    if (vxbIntConnect (pDev, pTimer->pIntRes, bcmSystTimerInt, pTFunc) != OK)
       {
       DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Failed to connect interrupt.\n");
       goto attachFail;
       }

    vxbTimerRegister (pTFunc);

    /* initialize the system Module spinlock */

    SPIN_LOCK_ISR_INIT (&pTimer->spinLock, 0);

    vxbDevSoftcSet (pDev, (void *) pTimer);

    /* clear the interrupt status for default channel of system timer Module */
    BCM_TIMER_REG_SETBIT_4 (
                 pTimer,
                 BCM_SYSTIMER_CS_OFFSET,
                 BCM_SYSTIMER_ENABLE_CHL(BCM_SYSTIMER_CHANNEL_DEFAULT));

    return OK;

attachFail:

    DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Failed attaching %s\n", __FUNCTION__);

    if (pTimer != NULL)     /* if pTModule allocate success */
        {
        vxbMemFree (pTimer);
        }
    (void) vxbClkDisableAll (pDev);
    vxbDevSoftcSet (pDev, NULL);

    return ERROR;
    }

/*******************************************************************************
*
* bcmSystTimerAllocate - allocate resources for a timer
*
* This is the function called to allocate a timer for usage by the timer
* abstraction layer.
*
* RETURNS: OK or ERROR if timer allocation failed.
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerAllocate
    (
    void * pCookie,
    UINT32 flags
    )
    {
    BCM_SYSTIMER_MODULE *          pTimer;
    struct vxbTimerFunctionality * pTimerFunc;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* validate the parameters */

    if (pCookie == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "pCookie is NULL.\n");
        return ERROR;
        }

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL.\n");
        return ERROR;
        }

    SPIN_LOCK_ISR_TAKE (&pTimer->spinLock);

    /* if the timer device is already allocated return ERROR */

    if (pTimer->pTimerFunc.allocated)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer is already allocated.\n");
        SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);
        return ERROR;
        }

    pTimer->pIsrFunc = NULL;
    pTimer->arg = (_Vx_usr_arg_t) NULL;

    /* set the timer allocated flag */

    pTimer->pTimerFunc.allocated = TRUE;

    SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);
    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerRelease - release the timer resource
*
* This is the function called to release a timer device.
*
* RETURNS: OK, or ERROR if failed to release a timer.
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerRelease
    (
    void * pCookie
    )
    {
    struct vxbTimerFunctionality *      pTimerFunc;
    BCM_SYSTIMER_MODULE *               pTimer;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* validate the parameters */

    if (pCookie == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "pCookie is NULL.\n");
        return ERROR;
        }

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL.\n");
        return ERROR;
        }

    SPIN_LOCK_ISR_TAKE (&pTimer->spinLock);

    if (!pTimer->pTimerFunc.allocated)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer not allocated.\n");
        SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);
        return ERROR;
        }

    /*  Clear Interrupt flag */

    BCM_TIMER_REG_SETBIT_4 (
                       pTimer,
                       BCM_SYSTIMER_CS_OFFSET,
                       BCM_SYSTIMER_ENABLE_CHL (BCM_SYSTIMER_CHANNEL_DEFAULT));

    pTimer->pIsrFunc = NULL;
    pTimer->arg = (_Vx_usr_arg_t) NULL;

    /* reset the timer allocated flag */

    pTimer->pTimerFunc.allocated = FALSE;

    SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);
    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerRolloverGet - retrieve the maximum value of the counter
*
* This is the function called to retrieve the maximum value of the counter.
* The maximum value is returned in <pCount> parameter.
*
* RETURNS: OK or ERROR if the parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerRolloverGet
    (
    void *   pCookie,
    UINT32 * pCount
    )
    {
    BCM_SYSTIMER_MODULE *               pTimer;
    struct vxbTimerFunctionality *      pTimerFunc;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* validate the parameters */

    if (pCookie == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "pCookie is NULL.\n");
        return ERROR;
        }

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL.\n");
        return ERROR;
        }

    SPIN_LOCK_ISR_TAKE (&pTimer->spinLock);
    *pCount = pTimer->maxCount;
    SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);

    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerCountGet - retrieve the current value of the counter
*
* This function is used to retrieve the current value of the counter.
* The current value is returned in 'pCount' parameter.
*
* RETURNS: OK or ERROR if the parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerCountGet
    (
    void *   pCookie,
    UINT32 * pCount
    )
    {
    BCM_SYSTIMER_MODULE *           pTimer;
    struct vxbTimerFunctionality *  pTimerFunc;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* validate the parameters */

    if (pCookie == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "pCookie is NULL.\n");
        return ERROR;
        }

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL\n");
        return ERROR;
        }

    *pCount = BCM_TIMER_REG_READ (pTimer, \
                                  BCM_SYSTIMER_CLO_OFFSET);

    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerISRSet - set a function to be called on the timer interrupt
*
* This function is called to set a function which can be called whenever
* the timer interrupt occurs.
*
* RETURNS: OK or ERROR if the parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerISRSet
    (
    void *        pCookie,
    void          (*pFunc)(_Vx_usr_arg_t),
    _Vx_usr_arg_t arg
    )
    {
    BCM_SYSTIMER_MODULE *          pTimer;
    struct vxbTimerFunctionality * pTimerFunc;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* validate the parameters */

    if (pCookie == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "pCookie is NULL.\n");
        return ERROR;
        }

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL.\n");
        return ERROR;
        }

    /* take the spinlock to update pIsrFunc and arg atomically */

    SPIN_LOCK_ISR_TAKE (&pTimer->spinLock);

    /* store the interrupt routine and argument information */

    pTimer->pIsrFunc = pFunc;
    pTimer->arg = arg;

    /* release the spinlock */

    SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);

    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerDisable - disable the timer channel and clear interrupt flag
*
* This function is called to disable the timer channel and clear interrupt flag
*
* RETURNS: OK or ERROR if timer is not disabled
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerDisable
    (
    void * pCookie
    )
    {
    struct vxbTimerFunctionality * pTimerFunc;
    BCM_SYSTIMER_MODULE *          pTimer;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* validate the parameters */

    if (pCookie == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "pCookie is NULL.\n");
        return ERROR;
        }

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL.\n");
        return ERROR;
        }

    SPIN_LOCK_ISR_TAKE (&pTimer->spinLock);

    /* Clear Interrupt flag */

    BCM_TIMER_REG_SETBIT_4 (
                      pTimer,
                      BCM_SYSTIMER_CS_OFFSET,
                      BCM_SYSTIMER_ENABLE_CHL (BCM_SYSTIMER_CHANNEL_DEFAULT));

    pTimer->maxCount = BCM_SYSTIMER_MAX_COUNT;
    pTimer->isEnabled = FALSE;

    SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);

    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerEnable - enable the timer and timer's interrupt
*
* This function enables the timer and timer's interrupt.
*
* RETURNS: OK or ERROR if timer is not enabled
*
* ERRNO: N/A
*/

LOCAL STATUS bcmSystTimerEnable
    (
    void * pCookie,
    UINT32 maxTimerCount
    )
    {
    struct vxbTimerFunctionality * pTimerFunc;
    UINT32                         cntValue;
    BCM_SYSTIMER_MODULE *          pTimer;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    /* check whether the parameters are valid */

    if ((pCookie == NULL) || (maxTimerCount == 0) ||
        (maxTimerCount > BCM_SYSTIMER_MAX_COUNT))
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Wrong parameters:%s.\n", __FUNCTION__);
        return ERROR;
        }

    /* retrieve the pTimer */

    pTimerFunc = (struct vxbTimerFunctionality *) pCookie;
    pTimer = TIMERFUNC_TO_TIMERDATA (pTimerFunc);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer data is NULL\n");
        return ERROR;
        }

    SPIN_LOCK_ISR_TAKE (&pTimer->spinLock);

    /* Update STM CMP Register using max count value */

    cntValue = BCM_TIMER_REG_READ (pTimer, BCM_SYSTIMER_CLO_OFFSET);
    pTimer->cmpStartValue = cntValue;

    BCM_TIMER_REG_WRITE (pTimer,
                         BCM_SYSTIMER_CHL_CMP(BCM_SYSTIMER_CHANNEL_DEFAULT),
                         cntValue + maxTimerCount);

    pTimer->maxCount = maxTimerCount;
    pTimer->pTimerFunc.ticksPerSecond = pTimer->pTimerFunc.clkFrequency /
                                        maxTimerCount;

    (void) vxbIntEnable (pTimer->pDev, pTimer->pIntRes);

    pTimer->isEnabled = TRUE;

    SPIN_LOCK_ISR_GIVE (&pTimer->spinLock);

    return OK;
    }

/*******************************************************************************
*
* bcmSystTimerInt - ISR for the Timer
*
* This routine handles the Timer interrupt.
*
* RETURNS : N/A
*
* ERRNO: N/A
*/

LOCAL void bcmSystTimerInt
    (
    BCM_SYSTIMER_MODULE * pTimer
    )
    {
    UINT32              regValue;
    UINT32              cntValue;

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Enter %s.\n", __FUNCTION__);

    if (pTimer == NULL)
        {
        DEBUG_MSG (BCM_SYSTIMER_DBG_ERR, "Timer Functionality is NULL.\n");
        return;
        }

    regValue = BCM_TIMER_REG_READ (pTimer, BCM_SYSTIMER_CS_OFFSET);

    DEBUG_MSG (BCM_SYSTIMER_DBG_INFO, "Interrupt flag (%x)\n", regValue);

    if ((regValue & BCM_SYSTIMER_ENABLE_CHL(BCM_SYSTIMER_CHANNEL_DEFAULT))
         == BCM_SYSTIMER_ENABLE_CHL(BCM_SYSTIMER_CHANNEL_DEFAULT))
        {

        /* Update system timer CMP register */
        if (pTimer->isEnabled)
            {
            cntValue = BCM_TIMER_REG_READ (pTimer, BCM_SYSTIMER_CLO_OFFSET);
            pTimer->cmpStartValue = cntValue;

            BCM_TIMER_REG_WRITE (
                         pTimer,
                         BCM_SYSTIMER_CHL_CMP(BCM_SYSTIMER_CHANNEL_DEFAULT),
                         cntValue + pTimer->maxCount);
            }

        BCM_TIMER_REG_SETBIT_4 (
                     pTimer,
                     BCM_SYSTIMER_CS_OFFSET,
                     BCM_SYSTIMER_ENABLE_CHL(BCM_SYSTIMER_CHANNEL_DEFAULT));

        if ((pTimer->pIsrFunc != NULL) && (pTimer->isEnabled))
            {
            pTimer->pIsrFunc (pTimer->arg);
            }
        }
    }
