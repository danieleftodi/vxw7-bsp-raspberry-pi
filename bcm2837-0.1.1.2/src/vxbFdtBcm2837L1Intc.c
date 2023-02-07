/* vxbFdtBcm2837L1Intc.c - BCM2837 L1 interrupt controller driver */

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

This module implements the interrupt controller driver for the BCM2837 L1
Interrupt Controller.

This driver is bound to device tree, and the device tree node must specify
below parameters:

\is

\i <reg>
This property specifies the register base address and length of this module.

\i <compatible>
This property specifies the name of the interrupt controller driver. It must be
"brcm,bcm2837-l1-intc".

\i <#interrupt-cells>
This property specifies how many parameter it receives from an interrupt user
node. For this release of this driver, it can be 1 or 2.

If #interrupt-cells is 1, then the parameter is interrupt number. example:
interrupts = <8>; it means the interrupt number is 8.

If #interrupt-cells is 2, the first parameter is interrupt number, the second
is cpu index.

Example:
interrupts = <8 0>; it means the interrupt number is 8, bind to cpu 0.

\i <interrupt-controller>
This property specifies that this node is an interrupt controller.

\ie

An example is shown below:

\cs
local_intc: interrupt-controller@40000000
        {
        compatible = "brcm,bcm2837-l1-intc";
        reg = <0x0 0x40000000 0x0 0x100>;
        interrupt-controller;
        #interrupt-cells = <2>;
        interrupt-parent = <&local_intc>;
        };
timer: timer
    {
    compatible = "arm,arm-gen-timer";
    interrupt-parent = <&local_intc>;
    interrupts = <0 0>,
                 <1 0>,
                 <3 0>,
                 <2 0>;
    };
\ce

INCLUDE FILES: vxbIntLib.h

SEE ALSO:
\tb Devicetree Specification Release 0.1
*/

#include <vxWorks.h>
#include <vsbConfig.h>
#include <string.h>
#include <memLib.h>
#include <intLib.h>
#include <iv.h>
#include <arch/arm/intArmLib.h>
#include <arch/arm/vxAtomicArchLib.h>
#include <arch/arm/excArmLib.h>
#include <cpuset.h>
#include <stdlib.h>

#include <vxLib.h>
#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtLib.h>
#include <subsys/int/vxbIntLib.h>

#ifdef  _WRS_CONFIG_SV_INSTRUMENTATION
#   include <private/eventP.h>
#endif  /* _WRS_CONFIG_SV_INSTRUMENTATION */

/* defines */

/* debug macro */

#undef  INTC_DBG
#ifdef  INTC_DBG

/* turning local symbols into global symbols */

#ifdef  LOCAL
#undef  LOCAL
#define LOCAL
#endif

#include <private/kwriteLibP.h>    /* _func_kprintf */

#define INTC_DBG_OFF          0x00000000
#define INTC_DBG_ISR          0x00000001
#define INTC_DBG_ERR          0x00000002
#define INTC_DBG_INFO         0x00000004
#define INTC_DBG_ALL          0xffffffff

LOCAL UINT32 l1IntcDbgMask = INTC_DBG_ALL;

#define INTC_DBG(mask, ...)                                         \
    do                                                              \
        {                                                           \
        if ((l1IntcDbgMask & (mask)) || ((mask) == INTC_DBG_ALL))   \
            {                                                       \
                if (_func_kprintf != NULL)                          \
                    {                                               \
                    (* _func_kprintf)(__VA_ARGS__);                 \
                    }                                               \
                }                                                   \
            }                                                       \
    while ((FALSE))
#else
#define INTC_DBG(...)
#endif  /* INTC_DBG */

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#    define SWAP64 vxbSwap64
#else
#    define SWAP32
#    define SWAP64
#endif /* ARMBE8 */

#define INTR_EXC_ID (EXC_OFF_IRQ) /* exception id, for external interrupt */

#ifndef EXC_CONNECT_INTR_RTN
#   define EXC_CONNECT_INTR_RTN(rtn) \
    excIntConnect ((VOIDFUNCPTR *)INTR_EXC_ID, rtn)
#endif /* EXC_CONNECT_INTR_RTN */

/* this sets the CPU interrupt enable on */

#ifndef CPU_INTERRUPT_ENABLE
#   define CPU_INTERRUPT_ENABLE intCpuUnlock(0)
#endif /* CPU_INTERRUPT_ENABLE */

/* this resets the CPU interrupt enable */

#ifndef CPU_INTERRUPT_DISABLE
#   define CPU_INTERRUPT_DISABLE intCpuLock()
#endif /* CPU_INTERRUPT_DISABLE */

#define INTC_HDL(pCtrl)         (pCtrl)->intcHdl
#define INTC_BAR(pCtrl)         (pCtrl)->intcBase

#define INTC_READ_4(pCtrl, addr)        \
    SWAP32 (vxbRead32 (INTC_HDL(pCtrl), \
                       (UINT32 *) ((char *) INTC_BAR(pCtrl) + addr)))

#define INTC_WRITE_4(pCtrl, addr, data) \
    vxbWrite32 (INTC_HDL(pCtrl),        \
                (UINT32 *) ((char *) INTC_BAR(pCtrl) + addr), SWAP32(data))

#define INTC_READ_8(pCtrl, addr)        \
    SWAP64 (vxbRead64 (INTC_HDL(pCtrl), \
                       (UINT64 *) ((char *) INTC_BAR(pCtrl) + addr)))

#define INTC_WRITE_8(pCtrl, addr, data) \
    vxbWrite64 (INTC_HDL(pCtrl),        \
                (UINT64 *) ((char *) INTC_BAR(pCtrl) + addr), SWAP64(data))

#define IVEC_TO_INUM(intVec)    ((int) (intVec))
#define INUM_TO_IVEC(intNum)    ((VOIDFUNCPTR *) (intNum))

/* interrupt number */

#define LOCAL_INTC_CNTPSIRQ      0
#define LOCAL_INTC_CNTPNSIRQ     1
#define LOCAL_INTC_CNTHPIRQ      2
#define LOCAL_INTC_CNTVIRQ       3
#define LOCAL_INTC_MAILBOX0      4
#define LOCAL_INTC_MAILBOX1      5
#define LOCAL_INTC_MAILBOX2      6
#define LOCAL_INTC_MAILBOX3      7
#define LOCAL_INTC_GPU_PERI      8
#define LOCAL_INTC_PM            9
#define LOCAL_INTC_AXI           10
#define LOCAL_INTC_LOCALTIMER    11
#define LOCAL_INTC_COUNT         (LOCAL_INTC_LOCALTIMER + 1)

/* register */

/* Mailbox0 is used for CPC. */

#define LOCAL_MAILBOX0_SWITCH(cpuId)    (0x50 + (0x04 * (cpuId)))
#define LOCAL_MAILBOX0_SET(cpuId)       (0x80 + (0x10 * (cpuId)))
#define LOCAL_MAILBOX0_CLR(cpuId)       (0xc0 + (0x10 * (cpuId)))
#define LOCAL_MAILBOX0_IRQENABLE        0x01

/* GPU */

#define LOCAL_GPU_ROUTING               0x0c

/* core timer */

#define LOCAL_TIMER_SWITCH(cpuId)       (0x40 + (0x04 * (cpuId)))
#define LOCAL_TIMER_CNTPSIRQ_IRQ        (1 << 0)
#define LOCAL_TIMER_CNTPNSIRQ_IRQ       (1 << 1)
#define LOCAL_TIMER_CNTHPIRQ_IRQ        (1 << 2)
#define LOCAL_TIMER_CNTVIRQ_IRQ         (1 << 3)

#define LOCAL_IRQ_PENDING(cpuId)        (0x60 + (0x04 * (cpuId)))
#define LOCAL_CONTROL                   (0x00)
#define LOCAL_CORETIMER_PRESCALER       (0x08)

#define BCM2837_L1_INTC_IPI_COUNT       8

/* structure holding Generic Interrupt Controller in details */

typedef struct localIntInfo
    {
    UINT32          cpuIdx;
    } LOCAL_INT_INFO;

typedef struct bcm2837L1IntcDrvCtrl
    {
    VXB_DEV_ID      pInst;
    VIRT_ADDR       intcBase;
    void *          intcHdl;
    VXB_RESOURCE *  intcResMem;
    UINT32          bootCpuIdx;
    UINT32          intBase;
    UINT32          intcLinesNum;
    UINT32          intcIpiNum;
    UINT32          intcCpuNum;
    LOCAL_INT_INFO  localIntInfo[LOCAL_INTC_COUNT];
    } BCM2837_L1_INTC_DRV_CTRL;

/* forward declarations */

#ifdef _WRS_CONFIG_SMP
IMPORT UINT32   sysCpuAvailableGet (void);
#endif /* _WRS_CONFIG_SMP */

IMPORT UINT32   vxCpuIndexGet (void);

LOCAL STATUS    vxbBcm2837L1IntcEnableMethod (VXB_DEV_ID pDev, UINT32 vector,
                                              VXB_INTR_ENTRY * pVxbIntrEntry);
LOCAL STATUS    vxbBcm2837L1IntcDisableMethod (VXB_DEV_ID pDev, UINT32 vector,
                                               VXB_INTR_ENTRY * pVxbIntrEntry);
LOCAL STATUS    vxbBcm2837L1IntcBindMethod (VXB_DEV_ID pDev, UINT32 vector,
                                            cpuset_t destCpu,
                                            VXB_INTR_ENTRY * pVxbIntrEntry);
LOCAL STATUS    vxbBcm2837L1IntcConfigMethod (VXB_DEV_ID       pDev,
                                              UINT32           vector,
                                              VXB_INTR_ENTRY * pVxbIntrEntry);
LOCAL STATUS    fdtBcm2837L1IntcCtlrProbe (VXB_DEV_ID pDev);
LOCAL STATUS    fdtBcm2837L1IntcCtlrAttach (VXB_DEV_ID pDev);

#ifdef _WRS_CONFIG_SMP
LOCAL STATUS    vxbBcm2837L1IntcIpiGen (VXB_DEV_ID pCtlr, UINT32 vector,
                                        phys_cpuset_t cpus);
#endif /* _WRS_CONFIG_SMP */

LOCAL BCM2837_L1_INTC_DRV_CTRL * pVxbBcm2837L1IntcDrvCtrl = NULL;

LOCAL VXB_DRV_METHOD fdtBcm2837L1IntcCtlrMethodList[] =
    {
    /* DEVICE API */

    { VXB_DEVMETHOD_CALL(vxbDevProbe),      fdtBcm2837L1IntcCtlrProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach),     fdtBcm2837L1IntcCtlrAttach },
    { VXB_DEVMETHOD_CALL(vxbIntEnable),     vxbBcm2837L1IntcEnableMethod },
    { VXB_DEVMETHOD_CALL(vxbIntDisable),    vxbBcm2837L1IntcDisableMethod },
    { VXB_DEVMETHOD_CALL(vxbIntBind),       vxbBcm2837L1IntcBindMethod },
    { VXB_DEVMETHOD_CALL(vxbIntConfig),     vxbBcm2837L1IntcConfigMethod },
#ifdef _WRS_CONFIG_SMP
    { VXB_DEVMETHOD_CALL (vxbIntIpi),       vxbBcm2837L1IntcIpiGen },
#endif /* _WRS_CONFIG_SMP */

    { 0, NULL }
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY fdtBcm2837L1IntcMatch[] =
    {
        {
        "brcm,bcm2837-l1-intc",            /* compatible */
        NULL
        },
        {}                                 /* Empty terminated list */
    };

/* globals */

VXB_DRV vxbFdtBcm2837L1IntcCtlrDrv =
    {
    { NULL },
    "Bcm2837 L1 Intc",                  /* Name */
    "BCM 2837 L1 Intc FDT driver",      /* Description */
    VXB_BUSID_FDT,                      /* Class */
    0,                                  /* Flags */
    0,                                  /* Reference count */
    fdtBcm2837L1IntcCtlrMethodList      /* Method table */
    };

VXB_DRV_DEF(vxbFdtBcm2837L1IntcCtlrDrv);

#ifdef _WRS_CONFIG_SMP

/*****************************************************************************
*
* vxbBcm2837L1IntcIpiGen - generate Inter Processor Interrupt
*
* This routine generates Inter Processor Interrupt(IPI) through
* mailbox0.
*
* RETURNS: OK or ERROR if any parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcIpiGen
    (
    VXB_DEV_ID      pDev,
    UINT32          vector,
    phys_cpuset_t   cpus
    )
    {
    int                         ipiId = 0;
    UINT32                      index;
    BCM2837_L1_INTC_DRV_CTRL *  pCtrl = NULL;

    if (pDev == NULL)
        {
        return (ERROR);
        }
    pCtrl = vxbDevSoftcGet (pDev);
    if (pCtrl == NULL)
        {
        return (ERROR);
        }

    INTC_DBG (INTC_DBG_INFO,
              "vxbBcm2837L1IntcIpiGen vector %d, cpus 0x%x\n",
              vector, cpus);

    ipiId = (INT32) (vector - pCtrl->intcLinesNum);

    if (ipiId < 0 || (UINT32) ipiId >= pCtrl->intcIpiNum || cpus == 0)
        {
        return ERROR;
        }

    for(index = 0; index < pCtrl->intcCpuNum; index++)
        {
        if (CPUSET_ISSET (cpus, index))
            {
            INTC_WRITE_4 (pCtrl, LOCAL_MAILBOX0_SET (index), (1 << ipiId));
            }
        }

    return(OK);
    }

#endif /* _WRS_CONFIG_SMP */

/*******************************************************************************
*
* vxbBcm2837L1IntcISR - interrupt handler
*
* This is BCM2837 L1 ISR interrupt handler.
*
* RETURNS: N/A
*
* ERROR: N/A
*/

LOCAL void vxbBcm2837L1IntcISR (void)
    {
    int     vector;
    UINT32  value = 0;
    int     ipiId = 0;
    UINT32  cpuId = vxCpuIndexGet();
    BCM2837_L1_INTC_DRV_CTRL * pCtrl = pVxbBcm2837L1IntcDrvCtrl;
#ifdef _WRS_CONFIG_SV_INSTRUMENTATION
    int     loopCnt = -1;
#endif

    while (0 != (value = INTC_READ_4 (pCtrl, LOCAL_IRQ_PENDING (cpuId))))
        {
        vector = -1;

        INTC_DBG (INTC_DBG_ISR,
                  "vxbBcm2837L1IntcISR has new irq value 0x%x\n",
                  value);

        /* Loop until no more interrupts are found */

        if ((value & (1 << LOCAL_INTC_MAILBOX0)) != 0)
            {
            ipiId = ffsMsb (INTC_READ_4 (pCtrl, LOCAL_MAILBOX0_CLR (cpuId))) -
                    1;
            if (ipiId >= 0)
                {
                vector = (int) pCtrl->intcLinesNum + ipiId;
                INTC_WRITE_4 (pCtrl, LOCAL_MAILBOX0_CLR (cpuId), (1 << ipiId));
                }
            }
        else
            {
            vector = ffsMsb (value) - 1;
            }

        if ((vector < 0) ||
            ((UINT32) vector > (pCtrl->intcLinesNum + pCtrl->intcIpiNum)))
            {
            return;
            }

#ifdef _WRS_CONFIG_SV_INSTRUMENTATION

        WV_EVT_INT_ENT (vector)
        loopCnt++;

#endif  /* _WRS_CONFIG_SV_INSTRUMENTATION */

#ifdef _WRS_CONFIG_LP64
        VXB_INT_ISR_CALL ((UINT64) (pCtrl->intBase + (UINT32) vector));
#else
        VXB_INT_ISR_CALL ((pCtrl->intBase + (UINT32) vector));
#endif
        }

#ifdef _WRS_CONFIG_SV_INSTRUMENTATION
    while (loopCnt-- > 0)
        {
        EVT_CTX_0 (EVENT_INT_EXIT);
        }
#endif /* _WRS_CONFIG_SV_INSTRUMENTATION */
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcDevInit - initialize the interrupt controller
*
* This routine initializes the interrupt controller device.
*
* RETURNS: OK or ERROR if parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcDevInit
    (
    BCM2837_L1_INTC_DRV_CTRL * pCtrl
    )
    {
    UINT32 i = 0;

    /* Init core timer, the firmware sets CNTFRQ as 19.2Mhz */

    INTC_WRITE_4 (pCtrl, LOCAL_CONTROL, 0);
    INTC_WRITE_4 (pCtrl, LOCAL_CORETIMER_PRESCALER, 0x80000000);

    /* enalbe mailbox0 by default */

    for (i = 0; i < pCtrl->intcCpuNum; i++)
        {
        INTC_WRITE_4 (pCtrl, LOCAL_MAILBOX0_SWITCH (i),
                      LOCAL_MAILBOX0_IRQENABLE);
        }

    /* GPU peripheral interrupts are set to boot core by defatult */

    INTC_WRITE_4 (pCtrl, LOCAL_GPU_ROUTING,
                  pCtrl->localIntInfo[LOCAL_INTC_GPU_PERI].cpuIdx);

    return OK;
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcIntNumEnable - enable a single interrupt number
*
* This routine enables a specific interrupt number.
*
* RETURNS: OK or ERROR if number is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcIntNumEnable
    (
    BCM2837_L1_INTC_DRV_CTRL *  pCtrl,
    int                         intNum  /* interrupt number to be enabled */
    )
    {
    INTC_DBG (INTC_DBG_INFO,
              "vxbBcm2837L1IntcIntNumEnable %d\n",
              intNum);

    if (intNum < 0 ||
        (UINT32) intNum >= (pCtrl->intcLinesNum +  pCtrl->intcIpiNum))
        {
        return ERROR;
        }

    if ((intNum >= LOCAL_INTC_CNTPSIRQ) &&
        (intNum <= LOCAL_INTC_CNTVIRQ))
        {
        INTC_WRITE_4 (pCtrl,
                      LOCAL_TIMER_SWITCH (pCtrl->localIntInfo[intNum].cpuIdx),
                      (1 << intNum));
        }

    return OK;
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcIntNumDisable - disable a single interrupt number
*
* This routine disables a specific interrupt number.
*
* RETURNS: OK or ERROR if the number is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcIntNumDisable
    (
    BCM2837_L1_INTC_DRV_CTRL *  pCtrl,
    int                         intNum  /* interrupt number to be disabled */
    )
    {
    UINT32 value = 0;
    UINT32 reg = 0;

    INTC_DBG (INTC_DBG_INFO,
              "vxbBcm2837L1IntcIntNumDisable %d\n",
              intNum);

    if (intNum < 0 ||
        (UINT32) intNum >= (pCtrl->intcLinesNum +  pCtrl->intcIpiNum))
        {
        return ERROR;
        }

    if ((intNum >= LOCAL_INTC_CNTPSIRQ) &&
        (intNum <= LOCAL_INTC_CNTVIRQ))
        {
        reg = LOCAL_TIMER_SWITCH (pCtrl->localIntInfo[intNum].cpuIdx);
        value = INTC_READ_4 (pCtrl, reg);
        INTC_WRITE_4 (pCtrl, reg,  (value & ~(1 << intNum)));
        }

    return OK;
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcHwEnable - enable hardware interrupt
*
* This routine enables the interrupt for the configured input pin.
*
* RETURNS: OK if operation is successful else ERROR
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcHwEnable
    (
    int inputPin
    )
    {
    return vxbBcm2837L1IntcIntNumEnable (pVxbBcm2837L1IntcDrvCtrl, inputPin);
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcHwDisable - disable device interrupt
*
* This routine disables the interrupt for the configured input pin.
*
* RETURNS: OK if operation is successful else ERROR
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcHwDisable
    (
    int inputPin
    )
    {
    return vxbBcm2837L1IntcIntNumDisable (pVxbBcm2837L1IntcDrvCtrl, inputPin);
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcEnableMethod - vxbIntEnable method for BCM2837 L1 Interrupt
*                                Controller
*
* This routine enables the interrupt specified by the parameter - vector.
*
* RETURNS: OK if operation is successful else ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcEnableMethod
    (
    VXB_DEV_ID          pDev,
    UINT32              vector,
    VXB_INTR_ENTRY *    pVxbIntrEntry
    )
    {
    return vxbBcm2837L1IntcIntNumEnable (pVxbBcm2837L1IntcDrvCtrl,
                                         IVEC_TO_INUM (vector));
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcDisableMethod - vxbIntDisable method for BCM2837 L1 Interrupt
*                                 Controller
*
* This routine enables the interrupt specified by the parameter - vector.
*
* RETURNS: OK if operation is successful else ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcDisableMethod
    (
    VXB_DEV_ID          pDev,
    UINT32              vector,
    VXB_INTR_ENTRY *    pVxbIntrEntry
    )
    {
    return vxbBcm2837L1IntcIntNumDisable (pVxbBcm2837L1IntcDrvCtrl,
                                          IVEC_TO_INUM (vector));
    }

/*****************************************************************************
*
* vxbBcm2837L1IntcBindMethod - vxbIntBind method for BCM2837 L1 Interrupt
*                              Controller
*
* This routines binds the interrupt specified by vector to CPUs specified by
* destCpu.
*
* RETURNS: OK if operation is successful else ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcBindMethod
    (
    VXB_DEV_ID          pDev,
    UINT32              vector,
    cpuset_t            destCpu,
    VXB_INTR_ENTRY *    pVxbIntrEntry
    )
    {
#ifdef _WRS_CONFIG_SMP
    int     intNum   = IVEC_TO_INUM(vector);
    UINT32  bitCnt   = 0;
    UINT32  i        = 0;
    UINT32  cpu      = 0;
    phys_cpuset_t               physDestCpu;
    BCM2837_L1_INTC_DRV_CTRL *  pCtrl = NULL;

    if (pDev == NULL)
        {
        return ERROR;
        }

    /* get driver data */

    pCtrl = vxbDevSoftcGet (pDev);
    if (pCtrl == NULL)
        {
        return ERROR;
        }

    if (intNum < 0 ||
        (UINT32) intNum >= (pCtrl->intcLinesNum +  pCtrl->intcIpiNum))
        {
        return ERROR;
        }

    INTC_DBG (INTC_DBG_INFO,
              "vxbBcm2837L1IntcBindMethod vector %d cpubit %x\n",
              vector, destCpu);

    cpusetLogicalToPhys (destCpu, &physDestCpu);

    for (i = 0; i < pCtrl->intcCpuNum; i++)
        {
        if (CPUSET_ISSET (physDestCpu, i))
            {
            bitCnt++;
            cpu = i;
            }
        }

    if (bitCnt != 1)
        {
        return ERROR;
        }

    if ((intNum >= LOCAL_INTC_CNTPSIRQ) &&
        (intNum <= LOCAL_INTC_CNTVIRQ))
        {
        (void) vxbBcm2837L1IntcIntNumDisable (pCtrl, intNum);
        pCtrl->localIntInfo[intNum].cpuIdx = cpu;
        (void) vxbBcm2837L1IntcIntNumEnable (pCtrl, intNum);
        }
    else if (intNum == LOCAL_INTC_GPU_PERI)
        {
        pCtrl->localIntInfo[intNum].cpuIdx = cpu;
        INTC_WRITE_4 (pCtrl, LOCAL_GPU_ROUTING,
                      pCtrl->localIntInfo[intNum].cpuIdx);
        }

    return OK;
#else
    return ERROR;
#endif

    }

/*****************************************************************************
*
* vxbBcm2837L1IntcConfigMethod - vxbIntConfig method for BCM2837 L1 Interrupt
*
* This routine is used to configure the interrupt specified by vector.
* The configure data derives from "interrupts" property of the device node.
*
* RETURNS:  OK if operation is successful else ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837L1IntcConfigMethod
    (
    VXB_DEV_ID       pDev,
    UINT32           vector,
    VXB_INTR_ENTRY * pVxbIntrEntry
    )
    {
    BCM2837_L1_INTC_DRV_CTRL *  pCtrl;
    UINT32 *                    pProp   = NULL;
    int                         intNum  = IVEC_TO_INUM(vector);
    UINT32                      cpus    = 0;

    if (pDev == NULL)
        {
        return ERROR;
        }

    /* get driver data */

    pCtrl = vxbDevSoftcGet (pDev);
    if (pCtrl == NULL)
        {
        return ERROR;
        }

    if (pVxbIntrEntry == NULL || pVxbIntrEntry->numProp == 0)
        {
        /* do not use DTS to configure interrupt, by pass */

        INTC_DBG (INTC_DBG_INFO, "Old style DTS node @0x%x\n",
                  pVxbIntrEntry->node);
        return OK;
        }

    if (pVxbIntrEntry->numProp != 1)
        {
        INTC_DBG (INTC_DBG_ERR, "wrong cell value %d@0x%x\n",
                  pVxbIntrEntry->numProp, pVxbIntrEntry->node);
        return ERROR;
        }

    pProp = pVxbIntrEntry->pProp;
    cpus = vxFdt32ToCpu (*pProp);
    INTC_DBG (INTC_DBG_INFO, "vector %d cpus property is 0x%x\n",
              vector, cpus);

    if ((UINT32) intNum < pCtrl->intcLinesNum)
        {
        if ((intNum >= LOCAL_INTC_CNTPSIRQ) &&
            (intNum <= LOCAL_INTC_CNTVIRQ))
            {
            (void) vxbBcm2837L1IntcIntNumDisable (pCtrl, intNum);
            pCtrl->localIntInfo[intNum].cpuIdx = cpus;
            (void) vxbBcm2837L1IntcIntNumEnable (pCtrl, intNum);
            }
        else if (intNum == LOCAL_INTC_GPU_PERI)
            {
            pCtrl->localIntInfo[intNum].cpuIdx = vxFdt32ToCpu (*pProp);
            INTC_WRITE_4 (pCtrl, LOCAL_GPU_ROUTING,
                          pCtrl->localIntInfo[intNum].cpuIdx);
            }

        vxbIsrCpuSet ((int) (pVxbBcm2837L1IntcDrvCtrl->intBase + vector),
                      (cpuset_t) (1u << pCtrl->localIntInfo[intNum].cpuIdx));
        }

    return OK;
    }

/*******************************************************************************
*
* fdtBcm2837L1IntcCtlrProbe - probe for device presence at specific address
*
* Check for bcm2837 l1 interrupt controller (or compatible) device at the
* specified base address.
*
* RETURNS: OK if probe passes and assumed a valid (or compatible) device.
* ERROR otherwise.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcm2837L1IntcCtlrProbe
    (
    VXB_DEV_ID  pDev             /* device information */
    )
    {
    return vxbFdtDevMatch (pDev, fdtBcm2837L1IntcMatch, NULL);
    }

/*******************************************************************************
*
* fdtBcm2837L1IntcCtlrAttach - attach bcm2837 l1 intc device
*
* This is the bcm2837 l1 interrupt controller initialization routine.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcm2837L1IntcCtlrAttach
    (
    VXB_DEV_ID  pDev
    )
    {
    VXB_FDT_DEV *               pFdtDev;
    BCM2837_L1_INTC_DRV_CTRL *  pCtrl     = NULL;
    VXB_RESOURCE *              pResMem   = NULL;
    UINT32                      i;

    /* check for valid parameter */

    if (pDev == NULL)
        {
        return ERROR;
        }

    pFdtDev = (VXB_FDT_DEV *) (vxbDevIvarsGet (pDev));
    if (pFdtDev == NULL)
        {
        return ERROR;
        }

    pCtrl = (BCM2837_L1_INTC_DRV_CTRL *) \
            vxbMemAlloc (sizeof (BCM2837_L1_INTC_DRV_CTRL));
    if (pCtrl == NULL)
        {
        return ERROR;
        }

    /* set vxb device software context */

    vxbDevSoftcSet (pDev, (void *)pCtrl);
    pCtrl->pInst = pDev;

    pResMem = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);
    if ((pResMem == NULL) || (pResMem->pRes == NULL))
        {
        goto error;
        }

    pCtrl->intcBase = ((VXB_RESOURCE_ADR *) (pResMem->pRes))->virtual;
    pCtrl->intcHdl = (void *) ((VXB_RESOURCE_ADR *)
                               (pResMem->pRes))->pHandle;
    pCtrl->intcResMem = pResMem;
    INTC_DBG (INTC_DBG_INFO, "intcBase %llx, intcHdl %lld\n",
              (UINT64) pCtrl->intcBase, (UINT64) pCtrl->intcHdl);

    pCtrl->intcLinesNum = LOCAL_INTC_COUNT;
    pCtrl->bootCpuIdx = vxCpuIndexGet();
    for (i = 0; i < pCtrl->intcLinesNum; i++)
        {
        pCtrl->localIntInfo[i].cpuIdx = pCtrl->bootCpuIdx;
        }

#ifdef _WRS_CONFIG_SMP
    pCtrl->intcCpuNum = sysCpuAvailableGet ();
    pCtrl->intcIpiNum = BCM2837_L1_INTC_IPI_COUNT;
#else /* !_WRS_CONFIG_SMP */
    pCtrl->intcCpuNum = 1;
    pCtrl->intcIpiNum = 0;
#endif /* _WRS_CONFIG_SMP */

    /* install three pointers for legacy type intEnable/intDisable/intLevelSet */

    sysIntLvlEnableRtn  = (FUNCPTR) vxbBcm2837L1IntcHwEnable;
    sysIntLvlDisableRtn = (FUNCPTR) vxbBcm2837L1IntcHwDisable;
    sysIntLvlChgRtn     = (FUNCPTR) NULL;

    /*
     * save context to global variable because some interface are called without
     * providing the device handle, thus unable to retrieve context.
     */

    pVxbBcm2837L1IntcDrvCtrl = pCtrl;

    /* assign ISR to hook (defined in excArchLib.c) */

    (void) EXC_CONNECT_INTR_RTN (vxbBcm2837L1IntcISR);

    if (vxbIntRegister (pDev, (UINT32) pFdtDev->offset,
                        pCtrl->intcLinesNum,
                        pCtrl->intcIpiNum,
                        &pCtrl->intBase) == ERROR)
        {
        goto error;
        }

    if (vxbBcm2837L1IntcDevInit (pCtrl) != OK)
        {
        goto error;
        }

    INTC_DBG (INTC_DBG_INFO, "fdtBcm2837L1IntcCtlrAttach success\n");

    return OK;

error:

    if (pCtrl->intcResMem != NULL)
        {
        (void) vxbResourceFree (pDev, pCtrl->intcResMem);
        }
    if (pCtrl != NULL)
        {
        (void) vxbMemFree (pCtrl);
        }

    return ERROR;
    }

