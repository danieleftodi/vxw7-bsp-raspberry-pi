/* vxbFdtBcm2837Intc.c - BCM2837 interrupt controller driver */

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

This module implements the interrupt controller driver for the BCM2837 ArmCtrl
Interrupt Controller.

This driver is bound to device tree, and the device tree node must specify
below parameters:

\is

\i <reg>
This property specifies the register base address and length of this module.

\i <compatible>
This property specifies the name of the interrupt controller driver. It must be
"brcm,bcm2837-armctrl-ic".

\i <#interrupt-cells>
This property specifies how many parameter it receives from an interrupt user
node. For this release of this driver, it can be 1, and the parameter is
interrupt number.
example:
interrupts = <8>; it means the interrupt number is 8.

\i <interrupt-controller>
This property specifies that this node is an interrupt controller.

\i <interrupts>
This property specifies interrupt vector of the interrupts that are generated
by this device.

\i <interrupt-parent>
This property is available to define an interrupt parent.

\ie

An example is shown below:

\cs
intc: interrupt-controller@3f00b200
    {
    compatible = "brcm,bcm2837-armctrl-ic";
    reg = <0x0 0x3f00b200 0x0 0x200>;
    interrupt-controller;
    #interrupt-cells = <1>;
    interrupt-parent = <&local_intc>;
    interrupts = <8 0>;
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

LOCAL UINT32 intcDbgMask = INTC_DBG_ALL;

#define INTC_DBG(mask, ...)                                     \
    do                                                          \
        {                                                       \
        if ((intcDbgMask & (mask)) || ((mask) == INTC_DBG_ALL)) \
            {                                                   \
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

#define INTC_PENDING_BASIC          0x00
#define INTC_PENDING_IRQ1           0x04
#define INTC_PENDING_IRQ2           0x08
#define INTC_FIQ_CONTROL            0x0c
#define INTC_ENABLE_IRQ1            0x10
#define INTC_ENABLE_IRQ2            0x14
#define INTC_ENABLE_BASIC           0x18
#define INTC_DISABLE_IRQ1           0x1c
#define INTC_DISABLE_IRQ2           0x20
#define INTC_DISABLE_BASIC          0x24

#define INTC_BANK_NUM               3
#define INTC_BANK_INT_NUM           32
#define INTC_MAX_NUM                (INTC_BANK_INT_NUM * INTC_BANK_NUM)

#define INTC_PENDING_BASIC_IRQ1         8
#define INTC_PENDING_BASIC_IRQ2         9
#define INTC_PENDING_BASIC_MAP_START    10
#define INTC_PENDING_BASIC_MAP_END      20

/* structure holding Generic Interrupt Controller in details */

typedef struct localIntInfo
    {
    UINT32          cpuIdx;
    } LOCAL_INT_INFO;

typedef struct bcm2837IntcDrvCtrl
    {
    VXB_DEV_ID      pInst;
    VIRT_ADDR       intcBase;
    void *          intcHdl;
    VXB_RESOURCE *  intcResMem;
    VXB_RESOURCE *  intcIntRes;
    UINT32          intBase;
    UINT32          intcEnable[INTC_BANK_NUM];
    UINT32          intcDisable[INTC_BANK_NUM];
    UINT32          intcPend[INTC_BANK_NUM];
    } BCM2837_INTC_DRV_CTRL;

/* forward declarations */

LOCAL STATUS    vxbBcm2837IntcEnableMethod (VXB_DEV_ID pDev, UINT32 vector,
                                            VXB_INTR_ENTRY * pVxbIntrEntry);
LOCAL STATUS    vxbBcm2837IntcDisableMethod (VXB_DEV_ID pDev, UINT32 vector,
                                             VXB_INTR_ENTRY * pVxbIntrEntry);
LOCAL STATUS    fdtBcm2837IntcCtlrProbe (VXB_DEV_ID pDev);
LOCAL STATUS    fdtBcm2837IntcCtlrAttach (VXB_DEV_ID pDev);

LOCAL UINT32    bcm2837IntcBasicPendMap[] =
                {
                39, 41, 42, 50, 51,     /* band 1 */
                85, 86, 87, 88, 89, 94  /* band 2 */
                };

LOCAL VXB_DRV_METHOD fdtBcm2837IntcCtlrMethodList[] =
    {
    /* DEVICE API */

    { VXB_DEVMETHOD_CALL(vxbDevProbe),       fdtBcm2837IntcCtlrProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach),      fdtBcm2837IntcCtlrAttach },
    { VXB_DEVMETHOD_CALL(vxbIntEnable),      vxbBcm2837IntcEnableMethod },
    { VXB_DEVMETHOD_CALL(vxbIntDisable),     vxbBcm2837IntcDisableMethod },
    { 0, NULL }
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY fdtBcm2837IntcMatch[] =
    {
        {
        "brcm,bcm2837-armctrl-ic",         /* compatible */
        NULL
        },
        {}                                 /* Empty terminated list */
    };

/* globals */

VXB_DRV vxbFdtBcm2837IntcCtlrDrv =
    {
    { NULL },
    "Bcm2837 ArmCtrl Intc",             /* Name */
    "BCM 2837 ArmCtrl Intc FDT driver", /* Description */
    VXB_BUSID_FDT,                      /* Class */
    0,                                  /* Flags */
    0,                                  /* Reference count */
    fdtBcm2837IntcCtlrMethodList        /* Method table */
    };

VXB_DRV_DEF(vxbFdtBcm2837IntcCtlrDrv);

/*******************************************************************************
*
* vxbBcm2837IntcISR - interrupt handler
*
* This function is interrupt handler.
*
* RETURNS: N/A
*
* ERROR: N/A
*/

LOCAL void vxbBcm2837IntcISR
    (
    BCM2837_INTC_DRV_CTRL * pCtrl
    )
    {
    UINT32 value = 0;
    UINT32 bit = 0;
    UINT32 bankId = 0;

    while (1)
        {
        value = INTC_READ_4 (pCtrl, pCtrl->intcPend[0]);
        if (value == 0)
            {
            break;
            }

        INTC_DBG (INTC_DBG_ISR,
                  "vxbBcm2837IntcISR new irq 0x%x\n",
                  value);

        bit = (UINT32) ffsMsb (value) - 1;
        if ((bit >= INTC_PENDING_BASIC_MAP_START) &&
            (bit <= INTC_PENDING_BASIC_MAP_END))
            {
            bit = bcm2837IntcBasicPendMap[bit - INTC_PENDING_BASIC_MAP_START];

            if (vxbIntrVecs[pCtrl->intBase + bit]->flag &
                VXB_INTR_VEC_ENABLE)
                {
                VXB_INT_ISR_CALL ((ULONG) pCtrl->intBase + bit);
                }
            }
        else if ((bit == INTC_PENDING_BASIC_IRQ1) ||
                 (bit == INTC_PENDING_BASIC_IRQ2))
            {
            bankId = bit - INTC_PENDING_BASIC_IRQ1 + 1;
            value = INTC_READ_4 (pCtrl, pCtrl->intcPend[bankId]);

            if (value != 0)
                {
                bit = ((UINT32) ffsMsb (value) - 1) +
                      (INTC_BANK_INT_NUM * bankId);

                if (vxbIntrVecs[pCtrl->intBase + bit]->flag &
                    VXB_INTR_VEC_ENABLE)
                    {
                    VXB_INT_ISR_CALL ((ULONG) pCtrl->intBase + bit);
                    }
                }
            else
                {
                INTC_DBG (INTC_DBG_ISR,
                          "vxbBcm2837IntcISR weird no irq bank %d\n",
                          bankId);
                break;
                }
            }
        else
            {
            INTC_DBG (INTC_DBG_ISR,
                      "vxbBcm2837IntcISR unsupported irq 0x%x\n",
                      value);
            break;
            }
        }

    INTC_DBG (INTC_DBG_ISR,
              "vxbBcm2837IntcISR end of irq\n");
    }

/*****************************************************************************
*
* vxbBcm2837IntcEnableMethod - vxbIntEnable method for BCM2837 Interrupt
*                              Controller
*
* This routine enables the interrupt specified by the parameter - vector.
*
* RETURNS: OK if operation is successful else ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837IntcEnableMethod
    (
    VXB_DEV_ID          pDev,
    UINT32              vector,
    VXB_INTR_ENTRY *    pVxbIntrEntry
    )
    {
    int intNum = IVEC_TO_INUM (vector);
    BCM2837_INTC_DRV_CTRL *  pCtrl;

    if (pDev == NULL)
        {
        return ERROR;
        }
    pCtrl = vxbDevSoftcGet (pDev);
    if (pCtrl == NULL)
        {
        return ERROR;
        }

    INTC_DBG (INTC_DBG_INFO,
              "vxbBcm2837IntcEnableMethod vector %d\n",
              vector);

    if (intNum < 0 ||
        intNum >= INTC_MAX_NUM)
        {
        return ERROR;
        }

    INTC_WRITE_4 (pCtrl, pCtrl->intcEnable[vector / INTC_BANK_INT_NUM],
                  1 << (vector % INTC_BANK_INT_NUM));

    return OK;
    }

/*****************************************************************************
*
* vxbBcm2837IntcDisableMethod - vxbIntDisable method for BCM2837 Interrupt
*                               Controller
*
* This routine enables the interrupt specified by the parameter - vector.
*
* RETURNS: OK if operation is successful else ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837IntcDisableMethod
    (
    VXB_DEV_ID          pDev,
    UINT32              vector,
    VXB_INTR_ENTRY *    pVxbIntrEntry
    )
    {
    int intNum = IVEC_TO_INUM(vector);
    BCM2837_INTC_DRV_CTRL *  pCtrl;

    if (pDev == NULL)
        {
        return ERROR;
        }
    pCtrl = vxbDevSoftcGet (pDev);
    if (pCtrl == NULL)
        {
        return ERROR;
        }

    INTC_DBG (INTC_DBG_INFO,
              "vxbBcm2837IntcDisableMethod vector %d\n",
              vector);

    if (intNum < 0 ||
        intNum >= INTC_MAX_NUM)
        {
        return ERROR;
        }

    INTC_WRITE_4 (pCtrl, pCtrl->intcDisable[vector / INTC_BANK_INT_NUM],
                  1 << (vector % INTC_BANK_INT_NUM));

    return OK;
    }

/*******************************************************************************
*
* fdtBcm2837IntcCtlrProbe - probe for device presence at specific address
*
* Check for bcm2837 interrupt controller (or compatible) device at the
* specified base address.
*
* RETURNS: OK if probe passes and assumed a valid (or compatible) device.
* ERROR otherwise.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcm2837IntcCtlrProbe
    (
    VXB_DEV_ID  pDev             /* device information */
    )
    {
    return vxbFdtDevMatch (pDev, fdtBcm2837IntcMatch, NULL);
    }

/*******************************************************************************
*
* fdtBcm2837IntcCtlrAttach - attach bcm2837 intc device
*
* This is the bcm2837 interrupt controller initialization routine.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcm2837IntcCtlrAttach
    (
    VXB_DEV_ID  pDev
    )
    {
    VXB_FDT_DEV *               pFdtDev;
    BCM2837_INTC_DRV_CTRL *     pCtrl     = NULL;
    VXB_RESOURCE *              pResMem   = NULL;

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

    pCtrl = (BCM2837_INTC_DRV_CTRL *) \
            vxbMemAlloc (sizeof (BCM2837_INTC_DRV_CTRL));
    if (pCtrl == NULL)
        {
        return ERROR;
        }

    /* set vxb device software context */

    vxbDevSoftcSet (pDev, (void *) pCtrl);
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

    pCtrl->intcIntRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);
    if (pCtrl->intcIntRes == NULL)
        {
        INTC_DBG (INTC_DBG_ERR, "Failed to allocate intcIntRes\n");
        goto error;
        }

    pCtrl->intcEnable[0] = INTC_ENABLE_BASIC;
    pCtrl->intcEnable[1] = INTC_ENABLE_IRQ1;
    pCtrl->intcEnable[2] = INTC_ENABLE_IRQ2;
    pCtrl->intcDisable[0] = INTC_DISABLE_BASIC;
    pCtrl->intcDisable[1] = INTC_DISABLE_IRQ1;
    pCtrl->intcDisable[2] = INTC_DISABLE_IRQ2;
    pCtrl->intcPend[0] = INTC_PENDING_BASIC;
    pCtrl->intcPend[1] = INTC_PENDING_IRQ1;
    pCtrl->intcPend[2] = INTC_PENDING_IRQ2;

    /* disable all interrupts by default */

    INTC_WRITE_4 (pCtrl, INTC_DISABLE_BASIC, 0xF);
    INTC_WRITE_4 (pCtrl, INTC_DISABLE_IRQ1, 0xFFFFFFFF);
    INTC_WRITE_4 (pCtrl, INTC_DISABLE_IRQ2, 0xFFFFFFFF);

    /* disable FIQ by default */

    INTC_WRITE_4 (pCtrl, INTC_FIQ_CONTROL, 0);

    if (vxbIntRegister (pDev, (UINT32) pFdtDev->offset, INTC_MAX_NUM,
                        0, &pCtrl->intBase) == ERROR)
        {
        INTC_DBG (INTC_DBG_ERR, "Register legacy interrupt vectors failed\n");
        goto error;
        }

    if (vxbIntConnect (pDev, pCtrl->intcIntRes,
                       (VOIDFUNCPTR) vxbBcm2837IntcISR, pCtrl) != OK)
        {
        INTC_DBG (INTC_DBG_ERR, "vxbIntConnect intcIntRes failed\n");
        goto error;
        }

    if (vxbIntEnable (pDev, pCtrl->intcIntRes) != OK)
        {
        INTC_DBG (INTC_DBG_ERR, "vxbIntEnable intcIntRes failed\n");
        if (vxbIntDisconnect (pDev, pCtrl->intcIntRes) == ERROR)
            {
            INTC_DBG (INTC_DBG_ERR, "disconnect intcIntRes failed\n");
            }
        goto error;
        }

    INTC_DBG (INTC_DBG_INFO, "fdtBcm2837IntcCtlrAttach success\n");

    return OK;

error:

    if (pCtrl->intcIntRes != NULL)
        {
        (void) vxbResourceFree (pDev, pCtrl->intcIntRes);
        }
    if (pCtrl->intcResMem != NULL)
        {
        (void) vxbResourceFree(pDev, pCtrl->intcResMem);
        }
    if (pCtrl != NULL)
        {
        (void) vxbMemFree (pCtrl);
        }

    return ERROR;
    }

