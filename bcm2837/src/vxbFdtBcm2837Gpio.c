/* vxbFdtBcm2837Gpio.c - FDT Driver for Bcm2837 GPIO controller */

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
22apr19,zxw  removed ISR pointer check in vxbBcm2837GpioIntConfig()
08mar19,zxw  created (F11409)
*/

/*
DESCRIPTION

This is the vxBus driver for the pin in the Broadcom Bcm2837 GPIO module.

This driver provides callback method to support the GPIO library, and the
client users should use the API provided by the GPIO library to access the
low level hardware function.

To add the driver to the vxWorks image, add the following component to the
kernel configuration.

\cs
vxprj component add DRV_GPIO_FDT_BCM2837
\ce

This driver is bound to device tree, and the device tree node must specify
below parameters:

\cs
compatible:
                This parameter specifies the name of the GPIO controller
                driver. It must be "brcm,bcm2837-gpio".

reg:
                This parameter specifies the register base address and length
                of this module.

interrupt-parent:
                This parameter specifies the offset of interrupt controller.

interrupts:
                This parameter specifies the interrupt number of this module.

\ce

An example of device node is shown below:

\cs
    gpio: gpio@3f200000
        {
        compatible = "brcm,bcm2837-gpio";
        reg = <0x0 0x3f200000 0x0 0xb4>;
        interrupts = <84>;
        interrupt-parent = <&intc>;
        #gpio-cells = <4>;
        };
\ce

INCLUDE FILES: vxBus.h vxbGpioLib.h vxbFdtLib.h

SEE ALSO: vxBus
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <intLib.h>
#include <errnoLib.h>
#include <errno.h>
#include <sioLib.h>
#include <ioLib.h>
#include <vxAtomicLib.h>
#include <stdio.h>
#include <string.h>
#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtLib.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <subsys/gpio/vxbGpioLib.h>
#include <subsys/clk/vxbClkLib.h>
#include <subsys/pinmux/vxbPinMuxLib.h>
#include "vxbFdtBcm2837Gpio.h"

/* defines */

/* debug macro */

#undef DEBUG_BCM2837_GPIO
#ifdef DEBUG_BCM2837_GPIO
#include <private/kwriteLibP.h>         /* _func_kprintf */

#define DBG_OFF             0x00000000
#define DBG_WARN            0x00000001
#define DBG_ERR             0x00000002
#define DBG_INFO            0x00000004
#define DBG_ALL             0xffffffff
LOCAL UINT32 dbgMask = DBG_ALL;

#undef DBG_MSG
#define DBG_MSG(mask,...)                                           \
    do                                                              \
    {                                                               \
    if ((dbgMask & (mask)) || ((mask) == DBG_ALL))                  \
        {                                                           \
        if (_func_kprintf != NULL)                                  \
            {                                                       \
            (* _func_kprintf)("%s,%d, ",__FUNCTION__,__LINE__);     \
            (* _func_kprintf)(__VA_ARGS__);                         \
            }                                                       \
        }                                                           \
    }while (0)
#else
#define DBG_MSG(...)
#endif  /* DEBUG_BCM2837_GPIO */

#undef  CSR_READ_4
#define CSR_READ_4(pDrvCtrl, addr)                                  \
        vxbRead32 ((pDrvCtrl->handle),                              \
                   (UINT32 *)((char *)(pDrvCtrl->baseAddr) + addr))

#undef  CSR_WRITE_4
#define CSR_WRITE_4(pDrvCtrl, addr, data)                           \
        vxbWrite32 ((pDrvCtrl->handle),                             \
                    (UINT32 *)((char *)(pDrvCtrl->baseAddr) + addr), data)

#undef  CSR_SETBIT_4
#define CSR_SETBIT_4(pCtrl, addr, data)                             \
        CSR_WRITE_4 (pCtrl, addr, CSR_READ_4 (pCtrl, addr) | (data))

#undef  CSR_CLRBIT_4
#define CSR_CLRBIT_4(pCtrl, addr, data)                             \
        CSR_WRITE_4 (pCtrl, addr, CSR_READ_4 (pCtrl, addr) & ~(data))

/* typedefs */

typedef struct bcm2837GpioIntNode
    {
    VOIDFUNCPTR     pIsr;       /* ISR */
    void *          pArg;       /* parameter */
    BOOL            bEnabled;
    } BCM2837_INT_NODE;

typedef struct bcm2837GpioDrvCtrl
    {
    VXB_GPIOCTRL                gpioCtrl;
    VXB_DEV_ID                  pInst;
    void*                       handle;         /* handle of vxbRead/vxbWrite */
    void*                       baseAddr;       /* controller's base */
    VXB_RESOURCE *              sharedIntRes;   /* interrupt resources */
    atomic32_t                  intEnCnt;       /* how many int enabled */
    BCM2837_INT_NODE *          pIntNode[BCM2837_GPIO_EINT_RANGE];
    INTR_TRIGER                 triggerType[BCM2837_GPIO_EINT_RANGE];
    spinlockIsr_t               spinLock;
    } BCM2837_GPIO_DRVCTRL;

/* forward declarations */

LOCAL STATUS fdtBcm2837GpioProbe (VXB_DEV_ID pDev);
LOCAL STATUS fdtBcm2837GpioAttach (VXB_DEV_ID pDev);
LOCAL void   vxbBcm2837GpioPudSet (BCM2837_GPIO_DRVCTRL * pDrvCtrl,
                                   UINT32 id,
                                   BCM2837_GPIO_PULL_TYPE pullMode);
LOCAL STATUS vxbBcm2837GpioReset (struct vxbGpioCtrl * pCtrl,
                                  UINT32 id);
LOCAL UINT32 vxbBcm2837GpioGetDir (struct vxbGpioCtrl * pCtrl,
                                   UINT32 id);
LOCAL STATUS vxbBcm2837GpioSetDir (struct vxbGpioCtrl * pCtrl,
                                   UINT32 id,UINT32 dir);
LOCAL UINT32 vxbBcm2837GpioGetValue (struct vxbGpioCtrl * pCtrl,
                                     UINT32 id);
LOCAL STATUS vxbBcm2837GpioSetValue (struct vxbGpioCtrl * pCtrl,
                                     UINT32 id, UINT32 value);
LOCAL STATUS vxbBcm2837GpioIntConfig (struct vxbGpioCtrl * pCtrl,
                                      UINT32 id, INTR_TRIGER trig,
                                      INTR_POLARITY pol);
LOCAL VOID   vxbBcm2837GpioISR (BCM2837_GPIO_DRVCTRL * pDrvCtrl);
LOCAL STATUS vxbBcm2837GpioIntConnect (struct vxbGpioCtrl * pCtrl,
                                       UINT32 id, VOIDFUNCPTR pIsr,
                                       void * pArg);
LOCAL STATUS vxbBcm2837GpioIntDisConnect (struct vxbGpioCtrl * pCtrl,
                                          UINT32 id, VOIDFUNCPTR pIsr,
                                          void * pArg);
LOCAL STATUS vxbBcm2837GpioIntEnable (struct vxbGpioCtrl * pCtrl,
                                      UINT32 id, VOIDFUNCPTR pIsr,
                                      void * pArg);
LOCAL STATUS vxbBcm2837GpioIntDisable (struct vxbGpioCtrl * pCtrl,
                                       UINT32 id, VOIDFUNCPTR pIsr,
                                       void * pArg);
#ifdef _WRS_CONFIG_DEBUG_FLAG
LOCAL VOID vxbBcm2837GpioShow (struct vxbGpioCtrl * pCtrl, UINT32 verbose);
#endif /* _WRS_CONFIG_DEBUG_FLAG */

/* locals */

LOCAL VXB_DRV_METHOD fdtBcm2837GpioMethodList[] =
    {
    /* DEVICE API */
    { VXB_DEVMETHOD_CALL(vxbDevProbe),  fdtBcm2837GpioProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach), fdtBcm2837GpioAttach },
    { 0, NULL }
    };

/* globals */

VXB_DRV vxbFdtBcm2837GpioDrv =
    {
    { NULL } ,
    "bcm2837-gpio",             /* Name */
    "BROADCOM GPIO controller", /* Description */
    VXB_BUSID_FDT,              /* Class */
    0,                          /* Flags */
    0,                          /* Reference count */
    fdtBcm2837GpioMethodList    /* Method table */
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY bcm2837GpioMatch[] =
    {
    {
    "brcm,bcm2837-gpio",       /* compatible */
    (void *) NULL,
    },
    {}                         /* Empty terminated list */
    };

VXB_DRV_DEF(vxbFdtBcm2837GpioDrv)

/*******************************************************************************
*
* fdtBcm2837GpioProbe - probe for device presence at specific address
*
* Check for Broadcom Bcm2837 GPIO contoller (or compatible) device at the
* specified base address. We assume one is present at that address, but we need
* to verify.
*
* RETURNS: OK if probe passes and assumed a valid Bcm2837 GPIO contoller
* (or compatible) device.  ERROR otherwise.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcm2837GpioProbe
    (
    VXB_DEV_ID pDev
    )
    {
    return vxbFdtDevMatch (pDev, bcm2837GpioMatch, NULL);
    }

/*******************************************************************************
*
* vxbBcm2837GpioPudSet - pull up/down GPIO pin
*
* This routine sets a pin as internal pull up/down.
*
* RETURNS: OK, or ERROR if failed to config
*
* ERRNO: N/A
*/

LOCAL void vxbBcm2837GpioPudSet
    (
    BCM2837_GPIO_DRVCTRL * pDrvCtrl,
    UINT32                 id,
    BCM2837_GPIO_PULL_TYPE pullMode
    )
    {
    UINT32 gpioIndex;
    UINT32 regAddr;

    if (id >= 32)
        {
        regAddr = BCM2837_GPPUDCLK1;
        }
    else
        {
        regAddr = BCM2837_GPPUDCLK0;
        }
    gpioIndex = 1u << (id % 32);

    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPPUD, (UINT32) pullMode);
    vxbUsDelay(1);
    CSR_WRITE_4 (pDrvCtrl, regAddr, gpioIndex);
    vxbUsDelay(1);
    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPPUD, (UINT32) BCM2837_GPIO_PULL_DISABLED);
    CSR_WRITE_4 (pDrvCtrl, regAddr, 0);
    }

/*******************************************************************************
*
* vxbBcm2837GpioReset - reset a GPIO pin
*
* This routine resets a pin: set direction to input, disable interrupt.
*
* RETURNS: OK, or ERROR if failed to reset
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioReset
    (
    struct vxbGpioCtrl * pCtrl,
    UINT32               id
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    UINT32                 regOffset;
    UINT32                 gpioIndex;

    regOffset = (id >> 5) * 4;
    gpioIndex = 1u << (id % 32);

    /*
     * Set as GPIO function and pull up.
     * Since no Pinmux Driver is provided now, GPIO driver implements part of
     * PinMux function here.
     */

    (void) vxbBcm2837GpioSetDir (pCtrl, id, GPIO_DIR_INPUT);

    SPIN_LOCK_ISR_TAKE (&pDrvCtrl->spinLock);

    vxbBcm2837GpioPudSet (pDrvCtrl, id, BCM2837_GPIO_PULL_UP);

    /* disable all interrupts */

    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPREN0  + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPFEN0  + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPHEN0  + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPLEN0  + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPAREN0 + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPAFEN0 + regOffset, gpioIndex);

    /* write to clear interrupt flag */

    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPEDS0 + regOffset, gpioIndex);
    SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);

    return OK;
    }

/*******************************************************************************
*
* fdtBcm2837GpioAttach - attach Broadcom Bcm2837 GPIO
*
* This is the Broadcom Bcm2837 GPIO initialization routine.
*
* RETURNS: OK, or ERROR if failed to attach GPIO
*
* ERRNO: N/A
*/

LOCAL STATUS fdtBcm2837GpioAttach
    (
    VXB_DEV_ID  pDev
    )
    {
    VXB_FDT_DEV *               pFdtDev = (VXB_FDT_DEV *) vxbFdtDevGet (pDev);
    VXB_GPIOCTRL *              pCtrl = NULL;
    BCM2837_GPIO_DRVCTRL *      pDrvCtrl = NULL;
    VXB_RESOURCE_ADR *          pResAdr = NULL;
    VXB_RESOURCE *              pRes;
    void *                      prop;
    int                         len;
    size_t                      size;

    if (pFdtDev == NULL)
        {
        DBG_MSG (DBG_ERR, "pFdtDev is NULL\n");
        goto ErrorAndFreeAll;
        }

    pDrvCtrl = vxbMemAlloc (sizeof (BCM2837_GPIO_DRVCTRL));
    if (pDrvCtrl == NULL)
        {
        DBG_MSG (DBG_ERR, "pDrvCtrl alloc failed\n");
        goto ErrorAndFreeAll;
        }

    pCtrl = &(pDrvCtrl->gpioCtrl);
    pDrvCtrl->pInst = pDev;
    pCtrl->pDev = pDev;

    /* get resources */

    pRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);
    if (pRes == NULL)
       {
       DBG_MSG (DBG_ERR, "reg resource alloc failed\n");
       goto ErrorAndFreeAll;
       }
    pResAdr = (VXB_RESOURCE_ADR *) pRes->pRes;
    if (pResAdr == NULL)
       {
       goto ErrorAndFreeAll;
       }
    pDrvCtrl->handle = pResAdr->pHandle;
    pDrvCtrl->baseAddr = (void *) (pResAdr->virtual);

    /* get interrupt resource */

    pRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);
    if (pRes == NULL)
        {
        DBG_MSG (DBG_ERR, "gpio shared irq resource alloc failed\n");
        goto ErrorAndFreeAll;
        }
    pDrvCtrl->sharedIntRes = pRes;

    /* alloc pValidBmp */

    pCtrl->length = BCM2837_GPIO_BANK_WIDTH;
    size = BCM2837_GPIO_BANK_WIDTH + GPIO_BIT_UNIT_SIZE - 1;
    size = (size / GPIO_BIT_UNIT_SIZE) * sizeof (UINT32);
    pCtrl->pValidBmp = vxbMemAlloc (size);
    if (pCtrl->pValidBmp == NULL)
        {
        DBG_MSG (DBG_ERR, "pValidBmp alloc failed.\n");
        goto ErrorAndFreeAll;
        }

    /* parse the GPIO cells */

    prop = (void*) vxFdtPropGet (pFdtDev->offset, "#gpio-cells", &len);
    if ((prop == NULL) || (len != 4))
        {
        DBG_MSG (DBG_ERR, "gpio-cells parse error,len=%d\n", len);
        goto ErrorAndFreeAll;
        }
    pCtrl->gpioCells = vxFdt32ToCpu (((UINT32*) prop)[0]);

    /*
     * Since GPIO lib will do generic allocating/free, so no need to add special
     * one here. Only ensure we are in known status.
     */

    pCtrl->gpioAlloc            = vxbBcm2837GpioReset;
    pCtrl->gpioFree             = vxbBcm2837GpioReset;
    pCtrl->gpioGetDir           = vxbBcm2837GpioGetDir;
    pCtrl->gpioSetDir           = vxbBcm2837GpioSetDir;
    pCtrl->gpioGetValue         = vxbBcm2837GpioGetValue;
    pCtrl->gpioSetValue         = vxbBcm2837GpioSetValue;
    pCtrl->gpioIntConnect       = vxbBcm2837GpioIntConnect;
    pCtrl->gpioIntDisConnect    = vxbBcm2837GpioIntDisConnect;
    pCtrl->gpioIntEnable        = vxbBcm2837GpioIntEnable;
    pCtrl->gpioIntDisable       = vxbBcm2837GpioIntDisable;
    pCtrl->gpioIntConfig        = vxbBcm2837GpioIntConfig;
#ifdef _WRS_CONFIG_DEBUG_FLAG
    pCtrl->gpioShow             = vxbBcm2837GpioShow;
#endif /* _WRS_CONFIG_DEBUG_FLAG */

    SPIN_LOCK_ISR_INIT (&pDrvCtrl->spinLock, 0);

    /* save Drvctrl */

    vxbDevSoftcSet (pDev, pDrvCtrl);

    /* connect int */

    if (vxbIntConnect (pDev, pDrvCtrl->sharedIntRes,
                       vxbBcm2837GpioISR, pDrvCtrl) == ERROR)
        {
        DBG_MSG (DBG_ERR, "interrupt connect failed\n");
        goto ErrorAndFreeAll;
        }

    if (vxbGpioAddCtlr (pCtrl) == OK)
        {
        return OK;
        }

ErrorAndFreeAll:
    if ((pCtrl != NULL) && (pCtrl->pValidBmp != NULL))
        {
        vxbMemFree (pCtrl->pValidBmp);
        }

    if (pDrvCtrl != NULL)
        {
        vxbDevSoftcSet (pDev, NULL);
        vxbMemFree (pDrvCtrl);
        }
    return ERROR;
    }

/*******************************************************************************
*
* vxbBcm2837GpioGetDir - get GPIO pin's direction
*
* This routine gets the specified GPIO pin's current direction.
*
* RETURNS: GPIO_DIR_INPUT or GPIO_DIR_INPUT
*
* ERRNO: N/A
*/

LOCAL UINT32 vxbBcm2837GpioGetDir
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    UINT32                 regOffset;
    UINT32                 gpioIndex;
    UINT32                 regValue;

    regOffset = (id / 10) * 4;
    gpioIndex = id % 10;

    regValue   = CSR_READ_4 (pDrvCtrl, BCM2837_GPFSEL0 + regOffset);
    regValue >>= gpioIndex * 3;
    regValue  &= BCM2837_GPIO_SEL_MASK;

    if (regValue == BCM2837_GPIO_INPUT_SEL)
        {
        return GPIO_DIR_INPUT;
        }
    else if (regValue == BCM2837_GPIO_OUTPUT_SEL)
        {
        return GPIO_DIR_OUTPUT;
        }
    else
        {
        (void) vxbBcm2837GpioReset (pCtrl, id);
        return GPIO_DIR_INPUT;
        }
    }

/*******************************************************************************
*
* vxbBcm2837GpioSetDir - set GPIO pin's direction
*
* This routine sets the speicific GPIO pin's direction.
*
* RETURNS: OK, or ERROR if failed to set direction
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioSetDir
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    UINT32                  dir
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    UINT32                 regOffset;
    UINT32                 gpioIndex;
    UINT32                 regValue;

    regOffset = (id / 10) * 4;
    gpioIndex = id % 10;

    SPIN_LOCK_ISR_TAKE (&pDrvCtrl->spinLock);
    regValue  = CSR_READ_4 (pDrvCtrl, BCM2837_GPFSEL0 + regOffset);
    regValue &= ~(BCM2837_GPIO_SEL_MASK << gpioIndex * 3);
    if (dir == GPIO_DIR_INPUT)
        {
        regValue |= BCM2837_GPIO_INPUT_SEL << gpioIndex * 3;
        }
    else
        {
        regValue |= BCM2837_GPIO_OUTPUT_SEL << gpioIndex * 3;
        }
    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPFSEL0 + regOffset, regValue);
    SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);

    return OK;
    }

/*******************************************************************************
*
* vxbBcm2837GpioGetValue - get GPIO pin's value
*
* This routine gets the specified pin's value.
*
* RETURNS: GPIO_VALUE_LOW or GPIO_VALUE_HIGH
*
* ERRNO: N/A
*/

LOCAL UINT32 vxbBcm2837GpioGetValue
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id
    )
    {
    UINT32                 regValue;
    UINT32                 gpioIndex;
    BCM2837_GPIO_DRVCTRL * pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;

    /*
     * Because bcm2837 doesn't have register to read back output value when
     * direction is output, here just return the real level of GPIO pin.
     */

    if (id < 32)
        {
        regValue = CSR_READ_4 (pDrvCtrl, BCM2837_GPLEV0);
        }
    else
        {
        regValue = CSR_READ_4 (pDrvCtrl, BCM2837_GPLEV1);
        }

    gpioIndex = 1u << (id % 32);
    regValue &= gpioIndex;

    if (regValue == 0)
        {
        return GPIO_VALUE_LOW;
        }
    else
        {
        return GPIO_VALUE_HIGH;
        }
    }

/*******************************************************************************
*
* vxbBcm2837GpioSetValue - set GPIO pin's value
*
* This routine sets the specified pin's value regardless it's output status.
*
* RETURNS: OK, or ERROR if failed to set value
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioSetValue
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    UINT32                  value
    )
    {
    UINT32                 gpioIndex;
    UINT32                 regOffset;
    BCM2837_GPIO_DRVCTRL * pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;

    regOffset = (id >> 5) * 4;
    gpioIndex = 1u << (id % 32);
    if (value == GPIO_VALUE_LOW)
        {
        CSR_WRITE_4 (pDrvCtrl, BCM2837_GPCLR0 + regOffset, gpioIndex);
        }
    else
        {
        CSR_WRITE_4 (pDrvCtrl, BCM2837_GPSET0 + regOffset, gpioIndex);
        }

    return OK;
    }

/*******************************************************************************
*
* vxbBcm2837GpioIntConfig - configure GPIO pin interrupt
*
* This routine configures interrupt trigger mode and polarity.
*
* RETURNS: OK, or ERROR if failed to config interrupt
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioIntConfig
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    INTR_TRIGER             trig,
    INTR_POLARITY           pol
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    UINT32                 gpioIndex;
    UINT32                 regOffset;

    regOffset = (id >> 5) * 4;
    gpioIndex = 1u << (id % 32);
    pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;

    SPIN_LOCK_ISR_TAKE (&pDrvCtrl->spinLock);

    /* enable specified interrupt */

    if (trig == INTR_TRIGGER_LEVEL)
        {
        pDrvCtrl->triggerType[id] = trig;
        if (pol == INTR_POLARITY_HIGH)
            {
            CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPLEN0 + regOffset, gpioIndex);
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPHEN0 + regOffset, gpioIndex);
            }
        else if (pol == INTR_POLARITY_LOW)
            {
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPLEN0 + regOffset, gpioIndex);
            CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPHEN0 + regOffset, gpioIndex);
            }
        else if (INTR_POLARITY_BOTH)
            {
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPLEN0 + regOffset, gpioIndex);
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPHEN0 + regOffset, gpioIndex);
            }
        else
            {
            SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);
            return OK;
            }
        }
    else if (trig == INTR_TRIGGER_EDGE)
        {
        pDrvCtrl->triggerType[id] = trig;
        if (pol == INTR_POLARITY_HIGH)
            {
            CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPFEN0 + regOffset, gpioIndex);
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPREN0 + regOffset, gpioIndex);
            }
        else if (pol == INTR_POLARITY_LOW)
            {
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPFEN0 + regOffset, gpioIndex);
            CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPREN0 + regOffset, gpioIndex);
            }
        else if (INTR_POLARITY_BOTH)
            {
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPFEN0 + regOffset, gpioIndex);
            CSR_SETBIT_4 (pDrvCtrl, BCM2837_GPREN0 + regOffset, gpioIndex);
            }
        else
            {
            SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);
            return OK;
            }
        }
    else
        {

        /* do not change */

        SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);
        return OK;
        }
    SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);

    /* clear pending interrupt */

    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPEDS0 + regOffset, gpioIndex);

    return OK;
    }

/*******************************************************************************
*
* vxbBcm2837GpioISR - GPIO interrupt handler
*
* This routine is GPIO interrupt handler.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL VOID vxbBcm2837GpioISR
    (
    BCM2837_GPIO_DRVCTRL * pDrvCtrl
    )
    {
    BCM2837_INT_NODE * pNode;
    UINT32             regValue;
    UINT32             regValue0;
    UINT32             regValue1;
    UINT32             regAddr;
    UINT32             bit;
    UINT32             ix;

    regValue0 = CSR_READ_4 (pDrvCtrl, BCM2837_GPEDS0);
    regValue1 = CSR_READ_4 (pDrvCtrl, BCM2837_GPEDS1);
    if ((regValue0 == 0) && (regValue1 == 0))
        {
        return;
        }

    for (ix = 0; ix < BCM2837_GPIO_EINT_RANGE; ix++)
        {
        if (ix < 32)
            {
            bit = 1u << ix;
            regValue = regValue0;
            regAddr = BCM2837_GPEDS0;
            }
        else
            {
            bit = 1u << (ix-31);
            regValue = regValue1;
            regAddr = BCM2837_GPEDS1;
            }

        if ((regValue & bit) != 0)
            {
            pNode = pDrvCtrl->pIntNode[ix];

            /*
             * clear edge interrupt flag before usr GPIO ISR, in case of losing
             * edge interrupt events.
             */

            if (pDrvCtrl->triggerType[ix] == INTR_TRIGGER_EDGE)
                {
                CSR_WRITE_4 (pDrvCtrl, regAddr, bit);
                }

            if ((pNode != NULL) && (pNode->bEnabled) && (pNode->pIsr != NULL))
                {
                pNode->pIsr (pNode->pArg);
                }
            else
                {

                /* suspicious interrupt */

                DBG_MSG (DBG_INFO, "suspicious interrupt %d\n", ix);
                }

            /*
             * clear level interrupt flag at last, in case of extra events
             * are triggered by same level.
             */

             if (pDrvCtrl->triggerType[ix] != INTR_TRIGGER_EDGE)
                {
                CSR_WRITE_4 (pDrvCtrl, regAddr, bit);
                }
            }
        }

    return;
    }

/*******************************************************************************
*
* vxbBcm2837GpioIntConnect - connect GPIO interrupt
*
* This routine connects GPIO interrupt to user supplied ISR.
*
* RETURNS: OK, or ERROR if failed to connect interrupt
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioIntConnect
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    VOIDFUNCPTR             pIsr,
    void *                  pArg
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl;
    BCM2837_INT_NODE *     pNode;

    pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    pNode    = pDrvCtrl->pIntNode[id];
    if (pNode == NULL)
        {
        pNode = vxbMemAlloc (sizeof (BCM2837_INT_NODE));
        if (pNode == NULL)
            {
            return ERROR;
            }
        pDrvCtrl->pIntNode[id] = pNode;
        }
    else
        {

        /* do not connect twice */

        DBG_MSG (DBG_ERR, "%d already connected\n", id);
        return ERROR;
        }

    pNode->bEnabled = FALSE;
    pNode->pIsr = pIsr;
    pNode->pArg = pArg;

    return OK;
    }

/*******************************************************************************
*
* vxbBcm2837GpioIntDisConnect - disconnect GPIO interrupt
*
* This routine disconnects GPIO interrupt from user supplied ISR.
*
* RETURNS: OK, or ERROR if failed to disconnect interrupt
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioIntDisConnect
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    VOIDFUNCPTR             pIsr,       /* ISR */
    void *                  pArg        /* parameter */
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl;
    BCM2837_INT_NODE *     pNode;

    pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    pNode    = pDrvCtrl->pIntNode[id];
    if (pNode == NULL)
        {

        /* not connected yet */

        return ERROR;
        }

    vxbMemFree (pNode);
    pDrvCtrl->pIntNode[id] = NULL;

    return OK;
    }

/*******************************************************************************
*
* vxbBcm2837GpioIntEnable - enable GPIO interrupt
*
* This routine enables the specified GPIO interrupt.
*
* RETURNS: OK, or ERROR if failed to enable interrupt
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioIntEnable
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    VOIDFUNCPTR             pIsr,
    void *                  pArg
    )
    {
    BCM2837_GPIO_DRVCTRL *  pDrvCtrl;
    BCM2837_INT_NODE *      pNode;
    STATUS                  status;

    pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;
    pNode = pDrvCtrl->pIntNode[id];
    if (pNode == NULL)
        {

        /* not connected yet */

        DBG_MSG (DBG_ERR, "%d not connected\n", id);
        return ERROR;
        }

    if (pNode->bEnabled)
        {
        return OK;
        }

    /* Enable GPIO pin's interrupt */

    SPIN_LOCK_ISR_TAKE (&pDrvCtrl->spinLock);
    if (vxAtomic32Get (&(pDrvCtrl->intEnCnt)) == 0)
        {

        /* the first one, enable the GPIO controller's int */

        status = vxbIntEnable (pCtrl->pDev, pDrvCtrl->sharedIntRes);
        if (status != OK)
            {
            DBG_MSG (DBG_ERR, "GPIO controller interrput enable failed.\n");
            SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);
            return ERROR;
            }
        }

    pNode->bEnabled = TRUE;
    (void) vxAtomic32Inc (&(pDrvCtrl->intEnCnt));
    SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);
    return OK;
    }

/*******************************************************************************
*
* vxbBcm2837GpioIntDisable - disable GPIO interrupt
*
* This routine disables the specified GPIO interrupt.
*
* RETURNS: OK, or ERROR if failed to disable interrupt
*
* ERRNO: N/A
*/

LOCAL STATUS vxbBcm2837GpioIntDisable
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  id,
    VOIDFUNCPTR             pIsr,
    void *                  pArg
    )
    {
    BCM2837_GPIO_DRVCTRL * pDrvCtrl;
    BCM2837_INT_NODE *     pNode;
    UINT32                 gpioIndex;
    UINT32                 regOffset;

    regOffset = (id >> 5) * 4;
    gpioIndex = 1u << (id % 32);

    pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;

    pNode = pDrvCtrl->pIntNode[id];

    if (pNode == NULL)
        {

        /* not connected yet */

        DBG_MSG (DBG_ERR, "%d not connected\n", id);
        return ERROR;
        }

    if (!pNode->bEnabled)
        {

        /* already disabled */

        return OK;
        }

    SPIN_LOCK_ISR_TAKE (&pDrvCtrl->spinLock);

    /* disable GPIO pin's interrupt */

    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPREN0 + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPFEN0 + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPHEN0 + regOffset, gpioIndex);
    CSR_CLRBIT_4 (pDrvCtrl, BCM2837_GPLEN0 + regOffset, gpioIndex);

    pNode->bEnabled = FALSE;

    if (vxAtomic32Get (&(pDrvCtrl->intEnCnt)) == 1)
        {
        /* the last one, disable the GPIO controller's int */

        (void) vxbIntDisable (pCtrl->pDev, pDrvCtrl->sharedIntRes);
        }

    (void) vxAtomic32Dec (&(pDrvCtrl->intEnCnt));
    SPIN_LOCK_ISR_GIVE (&pDrvCtrl->spinLock);
    return OK;
    }

#ifdef _WRS_CONFIG_DEBUG_FLAG
/*******************************************************************************
*
* vxbBcm2837GpioShow - show GPIO information
*
* This routine shows GPIO information.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL VOID vxbBcm2837GpioShow
    (
    struct vxbGpioCtrl *    pCtrl,
    UINT32                  verbose
    )
    {
    UINT32                 ix = 0;
    BCM2837_GPIO_DRVCTRL * pDrvCtrl;

    if (verbose == 0)
        {
        return;
        }

    pDrvCtrl = (BCM2837_GPIO_DRVCTRL *) pCtrl;

    (void) printf ("Raspberry Pi 3B+ GPIO controller information:\n");
    (void) printf ("====================GPIO Register information"
                   "================\n");

     for (ix = 0; ix <= 5; ix++)
        {
        (void) printf ("BCM2837_GPFSEL0%1d: 0x%08x\n", ix,
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPFSEL0 + ix * 4));
        }

        (void) printf ("BCM2837_GPPUD: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPPUD));

        (void) printf ("BCM2837_GPPUDCLK0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPPUDCLK0));
        (void) printf ("BCM2837_GPPUDCLK1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPPUDCLK1));

        (void) printf ("BCM2837_GPSET0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPSET0));
        (void) printf ("BCM2837_GPSET1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPSET1));

        (void) printf ("BCM2837_GPCLR0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPCLR0));
        (void) printf ("BCM2837_GPCLR1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPCLR1));

        (void) printf ("BCM2837_GPLEV0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPLEV0));
        (void) printf ("BCM2837_GPLEV1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPLEV1));

        (void) printf ("BCM2837_GPEDS0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPEDS0));
        (void) printf ("BCM2837_GPEDS1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPEDS1));

        (void) printf ("BCM2837_GPREN0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPREN0));
        (void) printf ("BCM2837_GPREN1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPREN1));

        (void) printf ("BCM2837_GPFEN0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPFEN0));
        (void) printf ("BCM2837_GPFEN1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPFEN1));

        (void) printf ("BCM2837_GPHEN0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPHEN0));
        (void) printf ("BCM2837_GPHEN1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPHEN1));

        (void) printf ("BCM2837_GPLEN0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPLEN0));
        (void) printf ("BCM2837_GPLEN1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPLEN1));

        (void) printf ("BCM2837_GPAREN0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPAREN0));
        (void) printf ("BCM2837_GPAREN1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPAREN1));

        (void) printf ("BCM2837_GPAFEN0: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPAFEN0));
        (void) printf ("BCM2837_GPAFEN1: 0x%08x\n",
                       CSR_READ_4 (pDrvCtrl, BCM2837_GPAFEN1));

    (void) printf ("====================GPIO interrupt information"
                   "===============\n");
    (void) printf ("pDrvCtrl at 0x%x, %d IRQs enabled\n", (UINT32) pDrvCtrl,
                   vxAtomic32Get (&(pDrvCtrl->intEnCnt)));
    for (ix = 0; ix < BCM2837_GPIO_EINT_RANGE; ix++)
        {
        if (pDrvCtrl->pIntNode[ix] != NULL)
            {
            (void) printf ("GPIO-(%d) interrupt infomation:\n", ix);
            (void) printf ("pIsr = 0x%08x, pArg = 0x%08x, \n",
                           (UINT32) (pDrvCtrl->pIntNode[ix]->pIsr),
                           (UINT32) (pDrvCtrl->pIntNode[ix]->pArg));
            if (pDrvCtrl->pIntNode[ix]->bEnabled)
                {
                (void) printf ("Enabled. \n");
                }
            else
                {
                (void) printf ("Disabled. \n");
                }
            }
        }

    return;
    }
#endif /* _WRS_CONFIG_DEBUG_FLAG */
