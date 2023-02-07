/* vxbFdtBcm2837Spi.c - FDT Driver for Bcm2837 SPI controller */

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
10oct19,zxw  fixed a data abort issue (V7PRO-5768)
08mar19,zxw  created (F11409)
*/

/*
DESCRIPTION

The Broadcom Bcm2837 SPI is a high-speed serial input/output port that allows
a serial bit stream to be shifted into and out of the device at a programmed
bit-transfer rate. The SPI is normally used for communication between the device
and external peripherals. Typical applications include interface to external I/O
or peripheral expansion via devices such as shift registers, display drivers,
SPI EPROMS, SPI Flash and analog-to-digital converters.

This module implements a driver for the SPI controller present on bcm2837 ARM
processor. The controller is capable of acting as a master mode. This
driver supports working on polling, interrupt mode, meanwhile, the driver can
dynamically switch clock frequency, duplex mode and chipSelect according to the
device requirement.

To add the driver to vxWorks image, add the following component to the
kernel configuration:

\cs
vxprj component add DRV_SPI_FDT_BCM2837
\ce

This driver is bound to device tree, and the device tree node must specify
below parameters:

\cs
compatible:
                This parameter specifies the name of the SPI controller
                driver. It must be "brcm,bcm2837-spi".

reg:
                This parameter specifies the register base address and length
                of this module.

interrupt-parent:
                This parameter specifies the offset of interrupt controller.

interrupts:
                This parameter specifies the interrupt number of this module.

clock-frequency:
                This parameter specifies the clock frequency provided by soc.

\ce

An example of device node is shown below:

\cs
    spi0: spi@3f204000
        {
        compatible = "brcm,bcm2837-spi";
        #address-cells = <1>;
        #size-cells = <0>;
        reg = <0x0 3f204000 0x0 0x1000>,
              <0x0 0x3f200000 0x0 0xb4>;
        interrupts = <86>;
        interrupt-parent = <&intc>;
        clock-frequency = <250000000>;
        status = "disabled";
    };
\ce

INCLUDE FILES: vxbSpiLib.h vxbFdtBcm2837Spi.h

SEE ALSO: vxBus, spiBus
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <semLib.h>
#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtSpiLib.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <subsys/clk/vxbClkLib.h>
#include "vxbFdtBcm2837Spi.h"

/* defines */

/* debug macro */

#undef DEBUG_BCM2837_SPI
#ifdef DEBUG_BCM2837_SPI
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
#endif  /* DEBUG_BCM2837_SPI */

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

#undef  SPI_PINMUX_READ_4
#define SPI_PINMUX_READ_4(pDrvCtrl, addr)                           \
        vxbRead32 ((pDrvCtrl->pinmuxHandle),                        \
                   (UINT32 *)((char *)(pDrvCtrl->pinmuxBaseAddr) + addr))

#undef  SPI_PINMUX_WRITE_4
#define SPI_PINMUX_WRITE_4(pDrvCtrl, addr, data)                    \
        vxbWrite32 ((pDrvCtrl->pinmuxHandle),                       \
                    (UINT32 *)((char *)(pDrvCtrl->pinmuxBaseAddr) + addr), data)

#undef  IS_SPI_FULL_DUPLEX
#define IS_SPI_FULL_DUPLEX(mode)  (((mode) & SPI_FULL_DUPLEX) != 0)

/* seconds to wait for SD/MMC command or data done */

#define SPI_CMD_WAIT_IN_SECS    10u
#define SPI_CMD_WAIT_EXTRA      (vxbSysClkRateGet () * 3u)

/* typedefs */

typedef struct bcm2837_spi_drv_ctrl
    {
    VXB_DEV_ID      pDev;
    void *          baseAddr;
    void *          handle;
    void *          pinmuxBaseAddr;
    void *          pinmuxHandle;
    UINT32          clkFrequency;
    UINT8 *         txBuf;
    UINT8 *         txBufEnd;
    UINT32          txLen;
    UINT8 *         rxBuf;
    UINT32          rxLen;
    UINT32          rxSkipLen;
    UINT32          curWorkingFrq;
    SEM_ID          semSync;
    VXB_RESOURCE *  intRes;
    VXB_RESOURCE *  spiRegRes;
    VXB_RESOURCE *  pinRegRes;
    } BCM2837_SPI_DRVCTRL;

/* forward declarations */

LOCAL STATUS bcm2837SpiProbe (VXB_DEV_ID pDev);
LOCAL STATUS bcm2837SpiAttach (VXB_DEV_ID pDev);
LOCAL void   bcm2837SpiReset (BCM2837_SPI_DRVCTRL * pDrvCtrl);
LOCAL void   bcm2837SpiInit (BCM2837_SPI_DRVCTRL * pDrvCtrl);
LOCAL void   bcm2837SpiIsr (BCM2837_SPI_DRVCTRL * pDrvCtrl);
LOCAL STATUS bcm2837SpiTransfer (VXB_DEV_ID pDev,
                                 SPI_HARDWARE * devInfo,
                                 SPI_TRANSFER * pPkg);
LOCAL STATUS bcm2837SpiTransferStart (VXB_DEV_ID pDev,
                                      SPI_HARDWARE * devInfo);
LOCAL STATUS bcm2837TransferDone (BCM2837_SPI_DRVCTRL * pDrvCtrl);
LOCAL void   bcm2837WriteFifo     (BCM2837_SPI_DRVCTRL * pDrvCtrl);
LOCAL void   bcm2837ReadFifo     (BCM2837_SPI_DRVCTRL * pDrvCtrl);
LOCAL STATUS bcm2837SpiTransferConfig (VXB_DEV_ID pDev,
                                       SPI_HARDWARE * devInfo);
LOCAL void   bcm2837ReadFifoFree (BCM2837_SPI_DRVCTRL * pDrvCtrl,
                                  UINT32 count);
LOCAL void   bcm2837WriteFifoFree (BCM2837_SPI_DRVCTRL * pDrvCtrl,
                                   UINT32 count);

/* locals */

LOCAL VXB_DRV_METHOD bcm2837SpiMethods[] = {
    { VXB_DEVMETHOD_CALL (vxbDevProbe),      (FUNCPTR) bcm2837SpiProbe},
    { VXB_DEVMETHOD_CALL (vxbDevAttach),     (FUNCPTR) bcm2837SpiAttach},
    { VXB_DEVMETHOD_CALL (vxbSpiXfer),       (FUNCPTR) bcm2837SpiTransfer},
    { VXB_DEVMETHOD_CALL (vxbFdtDevGet),     (FUNCPTR) vxbSpiFdtDevGet},
    { VXB_DEVMETHOD_CALL (vxbResourceFree),  (FUNCPTR) vxbSpiResFree},
    { VXB_DEVMETHOD_CALL (vxbResourceAlloc), (FUNCPTR) vxbSpiResAlloc},
    VXB_DEVMETHOD_END
};

VXB_DRV vxbFdtBcm2837SpiDrv =
    {
    { NULL } ,
    "bcm2837-spi",                          /* Name */
    "BROADCOM SPI controller",              /* Description */
    VXB_BUSID_FDT,                          /* Class */
    0,                                      /* Flags */
    0,                                      /* Reference count */
    (VXB_DRV_METHOD*) &bcm2837SpiMethods    /* Method table */
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY bcm2837SpiMatch[] =
    {
    {
    "brcm,bcm2837-spi",                 /* compatible */
    (void *)NULL
    },
    {}
    };

VXB_DRV_DEF (vxbFdtBcm2837SpiDrv)

/*******************************************************************************
*
* bcm2837SpiProbe - probe for device presence at specific address
*
* Check for Broadcom Bcm2837 SPI contoller (or compatible) device at the
* specified base address. We assume one is present at that address, but we need
* to verify.
*
* RETURNS: OK if probe passes and assumed a valid Bcm2837 SPI contoller
* (or compatible) device.  ERROR otherwise.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837SpiProbe
    (
    VXB_DEV_ID  pDev
    )
    {
    return vxbFdtDevMatch (pDev, bcm2837SpiMatch, NULL);
    }

/*******************************************************************************
*
* bcm2837SpiAttach - attach Broadcom Bcm2837 SPI controller
*
* This is the Broadcom Bcm2837 SPI controller initialization routine.
*
* RETURNS: OK, or ERROR if failed to attach SPI.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837SpiAttach
    (
    VXB_DEV_ID  pDev
    )
    {
    BCM2837_SPI_DRVCTRL *   pDrvCtrl;
    VXB_RESOURCE_ADR *      pResAdr = NULL;
    STATUS                  ret     = ERROR;
    VXB_CLK_ID              pClk;
    size_t                  size;

    if (pDev == NULL)
        {
        return ERROR;
        }

    size = sizeof (BCM2837_SPI_DRVCTRL);
    pDrvCtrl = (BCM2837_SPI_DRVCTRL *) vxbMemAlloc (size);
    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }
    pDrvCtrl->pDev = pDev;
    vxbDevSoftcSet (pDev, pDrvCtrl);

    /* get clock source */

    pClk = vxbClkGet (pDev, NULL);
     if (pClk != NULL)
         {
         (void) vxbClkEnable (pClk);
         }

    pDrvCtrl->clkFrequency = (UINT32) vxbClkRateGet (pClk);

    /* alloc resource */

    pDrvCtrl->spiRegRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);
    if (pDrvCtrl->spiRegRes == NULL)
        {
        DBG_MSG (DBG_ERR, "failed to get SPI reg resource.\n");
        goto attach_error;
        }

    pResAdr = (VXB_RESOURCE_ADR *) pDrvCtrl->spiRegRes->pRes;
    if (pResAdr == NULL)
        {
        goto attach_error;
        }
    pDrvCtrl->handle = pResAdr->pHandle;
    pDrvCtrl->baseAddr = (void *) pResAdr->virtual;

    pDrvCtrl->pinRegRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 1);
    if (pDrvCtrl->pinRegRes == NULL)
        {
         DBG_MSG (DBG_ERR, "failed to get SPI pin reg resource.\n");
        goto attach_error;
        }

    pResAdr = (VXB_RESOURCE_ADR *) pDrvCtrl->pinRegRes->pRes;
    if (pResAdr == NULL)
        {
        goto attach_error;
        }
    pDrvCtrl->pinmuxHandle = pResAdr->pHandle;
    pDrvCtrl->pinmuxBaseAddr = (void *) pResAdr->virtual;

    pDrvCtrl->intRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);
    if (pDrvCtrl->intRes == NULL)
        {
        DBG_MSG (DBG_ERR, "failed to alloc intRes.\n");
        goto attach_error;
        }

    bcm2837SpiInit (pDrvCtrl);

    /* connect and enable interrupt */

    if (pDrvCtrl->intRes != NULL)
        {

        /* The semSync semaphore is used to synchronize the SPI transfer. */

        pDrvCtrl->semSync = semBCreate (SEM_Q_PRIORITY, SEM_EMPTY);
        if (pDrvCtrl->semSync == SEM_ID_NULL)
            {
            DBG_MSG (DBG_ERR, "semBCreate failed for semSync\n");
            goto attach_error;
            }

        /* connect ISR and enable interrupt */

        ret = vxbIntConnect (pDev, pDrvCtrl->intRes, bcm2837SpiIsr, pDrvCtrl);
        if (ret != OK)
            {
            DBG_MSG (DBG_ERR, "failed to connect ISR.\n");
            goto attach_error;
            }

        ret = vxbIntEnable (pDev, pDrvCtrl->intRes);
        if (ret != OK)
            {
            DBG_MSG (DBG_ERR, "failed to enable interrupt.\n");
            (void) vxbIntDisconnect (pDev, pDrvCtrl->intRes);
            goto attach_error;
            }
        }

    if (vxbSpiCtrlRegister (pDev) != OK)
        {
        if (pDrvCtrl->intRes != NULL)
            {
            (void) vxbIntDisable (pDev, pDrvCtrl->intRes);
            (void) vxbIntDisconnect (pDev, pDrvCtrl->intRes);
            }
        DBG_MSG (DBG_ERR, "failed to register SPI controller.\n");
        goto attach_error;
        }

    return OK;

attach_error:
    if (pDrvCtrl->semSync != SEM_ID_NULL)
        {
        (void) semDelete (pDrvCtrl->semSync);
        }

    if (pDrvCtrl->intRes != NULL)
        {
        (void) vxbResourceFree (pDev, pDrvCtrl->intRes);
        }

    if (pDrvCtrl->spiRegRes != NULL)
        {
        (void) vxbResourceFree (pDev, pDrvCtrl->spiRegRes);
        }

    if (pDrvCtrl->pinRegRes != NULL)
        {
        (void) vxbResourceFree (pDev, pDrvCtrl->pinRegRes);
        }

    (void) vxbClkDisableAll (pDev);
    vxbDevSoftcSet (pDev, NULL);
    vxbMemFree (pDrvCtrl);
    return ERROR;
    }

/*******************************************************************************
*
* bcm2837SpiReset - reset SPI controller
*
* This routine resets Tx/Rx FIFO, chipSelect, interrupt and transfer of SPI
* controller.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837SpiReset
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl
    )
    {
    UINT32 regValue;

    regValue = CSR_READ_4 (pDrvCtrl, BCM2837_SPI_CS);

    /* clear tx/rx FIFO and disable CS */

    regValue |= BCM2837_SPI_CS_CLEAR_TX | BCM2837_SPI_CS_CLEAR_RX |
                BCM2837_SPI_CS_CS1      | BCM2837_SPI_CS_CS0;

    /* disable interrupts and set transfer inactive */

    regValue &= ~(BCM2837_SPI_CS_INTR  | BCM2837_SPI_CS_INTD |
                  BCM2837_SPI_CS_DMAEN | BCM2837_SPI_CS_TA);

    CSR_WRITE_4 (pDrvCtrl, BCM2837_SPI_CS, regValue);
    }

/*******************************************************************************
*
* bcm2837SpiInit - SPI controller initialization
*
* This routine performs the SPI controller initialization.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837SpiInit
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl
    )
    {
    UINT32  regValue;

    /*
     * set pinmux function:
     * set GPIO7-11 as CS1 CS0 MISO MOSI SCLK seperately.
     * set GPIO7-11 as pull up.
     */

    regValue  = SPI_PINMUX_READ_4 (pDrvCtrl, BCM2837_GPFSEL0);
    regValue &= ~((SPI_GPIO_BITS_MASK << SPI_CE0_BIT_OFFSET) |
                  (SPI_GPIO_BITS_MASK << SPI_CE1_BIT_OFFSET) |
                  (SPI_GPIO_BITS_MASK << SPI_MISO_BIT_OFFSET));
    regValue |=   (SPI_PINMUX_ALT0 << SPI_CE0_BIT_OFFSET) |
                  (SPI_PINMUX_ALT0 << SPI_CE1_BIT_OFFSET) |
                  (SPI_PINMUX_ALT0 << SPI_MISO_BIT_OFFSET);
    SPI_PINMUX_WRITE_4 (pDrvCtrl, BCM2837_GPFSEL0, regValue);

    regValue  = SPI_PINMUX_READ_4 (pDrvCtrl, BCM2837_GPFSEL0 + 4);
    regValue &= ~((SPI_GPIO_BITS_MASK << SPI_MOSI_BIT_OFFSET) |
                  (SPI_GPIO_BITS_MASK << SPI_SCLK_BIT_OFFSET));

    regValue |=   (SPI_PINMUX_ALT0 << SPI_MOSI_BIT_OFFSET) |
                  (SPI_PINMUX_ALT0 << SPI_SCLK_BIT_OFFSET);
    SPI_PINMUX_WRITE_4 (pDrvCtrl, BCM2837_GPFSEL0 + 4, regValue);

    regValue  = GPIO7_CLK_BIT_OFFSET | GPIO8_CLK_BIT_OFFSET  |
                GPIO9_CLK_BIT_OFFSET | GPIO10_CLK_BIT_OFFSET |
                GPIO11_CLK_BIT_OFFSET;
    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPPUD, BCM2837_GPIO_PULL_UP);
    vxbUsDelay (1);
    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPPUDCLK0, regValue);
    vxbUsDelay (1);
    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPPUD, (UINT32) BCM2837_GPIO_PULL_DISABLED);
    CSR_WRITE_4 (pDrvCtrl, BCM2837_GPPUDCLK0, 0);

    bcm2837SpiReset (pDrvCtrl);
    }

/*******************************************************************************
*
* bcm2837SpiIsr - interrupt service routine
*
* This routine handles interrupts of SPI.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837SpiIsr
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl
    )
    {
    UINT32 regValue;

    regValue = CSR_READ_4 (pDrvCtrl, BCM2837_SPI_CS);

    if (((regValue & BCM2837_SPI_CS_RXR) != 0))
        {
        bcm2837ReadFifoFree (pDrvCtrl, SPI_FIFO_RXR_LEN);
        }
    else if (((regValue & BCM2837_SPI_CS_RXF) != 0))
        {
        bcm2837ReadFifoFree (pDrvCtrl, SPI_FIFO_FULL_LEN);
        }

    if (bcm2837TransferDone (pDrvCtrl) == OK)
        {
        bcm2837WriteFifoFree (pDrvCtrl, SPI_FIFO_FULL_LEN);
        }

    /* read more data from FIFO */

    bcm2837ReadFifo (pDrvCtrl);

    /* write data to Tx FIFO */

    bcm2837WriteFifo (pDrvCtrl);

    if ((bcm2837TransferDone (pDrvCtrl) == OK) &&
        (pDrvCtrl->txLen == 0)                 &&
        (pDrvCtrl->rxLen == 0))
        {

        /*
         * complete Tx and Rx
         * By now, all the transmissions are completed. DONE interrupt must be
         * closed here in case of repeater trigger. In bcm2837SpiReset(), status
         * is reset and all interrupts are closed.
         */

        bcm2837SpiReset (pDrvCtrl);
        (void) semGive (pDrvCtrl->semSync);
        }
    }

/*******************************************************************************
*
* bcm2837SpiTransferConfig - configure SPI transfer parameters
*
* This routine configures the SPI registers which controls SPI master operation
* with its parameters.
*
* RETURNS: OK, or ERROR if failed to configure SPI transfer.
*
* ERRNO: N/A
*/

 LOCAL STATUS bcm2837SpiTransferConfig
    (
    VXB_DEV_ID      pDev,
    SPI_HARDWARE *  devInfo
    )
    {
    UINT32                  csConfig = 0;
    UINT32                  cdiv;
    BCM2837_SPI_DRVCTRL *   pDrvCtrl;

    pDrvCtrl = (BCM2837_SPI_DRVCTRL *) vxbDevSoftcGet (pDev);

    if (devInfo->chipSelect <= SPI_MAX_CS_NUM)
        {
        csConfig = devInfo->chipSelect;
        }

    /* calculate clock divisor */

    if (devInfo->devFreq == 0)
        {
        DBG_MSG (DBG_ERR, "device frequency is 0\n");
        return ERROR;
        }

    /*
     * SCLK = Core Clock / CDIV, CDIV must be an even number.
     * If CDIV is set to 0, the divisor is 65536.
     */

    cdiv = pDrvCtrl->clkFrequency / devInfo->devFreq;
    if (cdiv == 0)
        {
        cdiv = 2;
        }
    else if (pDrvCtrl->clkFrequency % devInfo->devFreq != 0)
        {
        cdiv++;
        }

    if (cdiv >= MAX_DIVIDERATIO)
        {
        cdiv = 0;
        }

    CSR_WRITE_4 (pDrvCtrl, BCM2837_SPI_CLK, cdiv);

    /* save the current working frequency */

    pDrvCtrl->curWorkingFrq = cdiv ? (pDrvCtrl->clkFrequency / cdiv) :
                                     (pDrvCtrl->clkFrequency / 65536);

    /* SPIEN polarity */

    if ((devInfo->mode & SPI_CSPOL_HIGH) != 0)
        {
        csConfig |= SPI_CS_ACTIVE_MASK << SPI_CE1_BIT_OFFSET;
        }
    else
        {
        csConfig &= ~(SPI_CS_ACTIVE_MASK << SPI_CE1_BIT_OFFSET);
        }

    /* SPI clock polarity */

    if ((devInfo->mode & SPI_CKPOL) != 0)
        {
        csConfig |= BCM2837_SPI_CS_CPOL;
        }
    else
        {
        csConfig &= ~BCM2837_SPI_CS_CPOL;
        }

    /* SPI clock phase */

    if ((devInfo->mode & SPI_CKPHA) != 0)
        {
        csConfig |= BCM2837_SPI_CS_CPHA;
        }
    else
        {
        csConfig &= ~BCM2837_SPI_CS_CPHA;
        }

    /* write the configuration to register */

    CSR_WRITE_4 (pDrvCtrl, BCM2837_SPI_CS, csConfig);

    return OK;
    }

/*******************************************************************************
*
* bcm2837ReadFifoFree - read SPI FIFO freely
*
* The routine reads specified count of bytes from FIFO without checking FIFO
* status.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837ReadFifoFree
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl,
    UINT32                count
    )
    {
    UINT8 readByte;

    if (count > pDrvCtrl->rxSkipLen + pDrvCtrl->rxLen)
        {
        count = pDrvCtrl->rxSkipLen + pDrvCtrl->rxLen;
        }

    while (count--)
        {
        readByte = (UINT8) CSR_READ_4 (pDrvCtrl, BCM2837_SPI_FIFO);
        if (pDrvCtrl->rxSkipLen > 0)
            {
            pDrvCtrl->rxSkipLen--;
            }
        else if(pDrvCtrl->rxLen > 0)
            {
            if (pDrvCtrl->rxBuf != NULL)
                {
                *pDrvCtrl->rxBuf++ = readByte;
                }
            pDrvCtrl->rxLen--;
            }
        }
    }

/*******************************************************************************
*
* bcm2837WriteFifoFree - write SPI FIFO freely
*
* The routine writes specified count of bytes from FIFO without checking FIFO
* status.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837WriteFifoFree
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl,
    UINT32                count
    )
    {
    UINT8 sendByte;

    if (pDrvCtrl->txLen < count)
        {
        count = pDrvCtrl->txLen;
        }

    while (count--)
        {
        if ((pDrvCtrl->txBufEnd != pDrvCtrl->txBuf) &&
            (pDrvCtrl->txBuf != NULL))
            {
            sendByte = *pDrvCtrl->txBuf++;
            }
        else
            {
            sendByte = 0;
            }
        pDrvCtrl->txLen--;
        CSR_WRITE_4 (pDrvCtrl, BCM2837_SPI_FIFO, (UINT32) sendByte);
        }
    }

/*******************************************************************************
*
* bcm2837ReadFifo - read SPI FIFO
*
* The routine reads as many bytes as possible from SPI FIFO.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837ReadFifo
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl
    )
    {
    UINT8 readByte;

    while ((CSR_READ_4 (pDrvCtrl, BCM2837_SPI_CS) & BCM2837_SPI_CS_RXD) != 0)
        {
        readByte = (UINT8) CSR_READ_4 (pDrvCtrl, BCM2837_SPI_FIFO);
        if (pDrvCtrl->rxSkipLen > 0)
            {
            pDrvCtrl->rxSkipLen--;
            }
        else if(pDrvCtrl->rxLen > 0)
            {
            if (pDrvCtrl->rxBuf != NULL)
                {
                *pDrvCtrl->rxBuf++ = readByte;
                }
            pDrvCtrl->rxLen--;
            }
        }
    }

/*******************************************************************************
*
* bcm2837WriteFifo - write SPI FIFO
*
* The routine writes as many bytes as possible to SPI FIFO.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837WriteFifo
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl
    )
    {
    UINT8 sendByte;

    while ((pDrvCtrl->txLen > 0) &&
            ((CSR_READ_4 (pDrvCtrl, BCM2837_SPI_CS) & BCM2837_SPI_CS_TXD) != 0))
        {
        if ((pDrvCtrl->txBufEnd != pDrvCtrl->txBuf) &&
            (pDrvCtrl->txBuf != NULL))
            {
            sendByte = *pDrvCtrl->txBuf++;
            }
        else
            {
            sendByte = 0;
            }
        pDrvCtrl->txLen--;
        CSR_WRITE_4 (pDrvCtrl, BCM2837_SPI_FIFO, (UINT32) sendByte);
        }
    }

/*******************************************************************************
*
* bcm2837TransferDone - check SPI transfer status
*
* The routine checks if the bytes in Tx FIFO are all sent out.
*
* RETURNS: OK, or ERROR if transfer is not done.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837TransferDone
    (
    BCM2837_SPI_DRVCTRL * pDrvCtrl
    )
    {
    if ((CSR_READ_4 (pDrvCtrl, BCM2837_SPI_CS) & BCM2837_SPI_CS_DONE) != 0)
        {
        return OK;
        }
    else
        {
        return ERROR;
        }
    }

/*******************************************************************************
*
* bcm2837SpiTransferStart - start SPI transfer
*
* The routine starts a transmission.
*
* RETURNS: OK, or ERROR if failed to transfer.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837SpiTransferStart
    (
    VXB_DEV_ID      pDev,
    SPI_HARDWARE *  devInfo
    )
    {
    BCM2837_SPI_DRVCTRL *   pDrvCtrl;
    UINT64                  timeoutTick;
    UINT32                  pollTimeout;
    UINT32                  pollCount;

    pDrvCtrl = (BCM2837_SPI_DRVCTRL *) vxbDevSoftcGet (pDev);

    if ((devInfo->mode & SPI_POLLING) == 0)
        {

        /* interrupt mode */

        CSR_SETBIT_4 (pDrvCtrl, BCM2837_SPI_CS, BCM2837_SPI_CS_TA);
        bcm2837WriteFifoFree (pDrvCtrl, SPI_FIFO_FULL_LEN);
        CSR_SETBIT_4 (pDrvCtrl, BCM2837_SPI_CS,
                      BCM2837_SPI_CS_INTR |
                      BCM2837_SPI_CS_INTD);

        timeoutTick  = SPI_CMD_WAIT_IN_SECS * vxbSysClkRateGet () *
                       sizeof (char) * (UINT64) (pDrvCtrl->txLen);
        timeoutTick /= pDrvCtrl->curWorkingFrq;
        timeoutTick += SPI_CMD_WAIT_EXTRA;
        if (semTake (pDrvCtrl->semSync, (UINT32) timeoutTick) != OK)
            {
            DBG_MSG (DBG_ERR, "interrupt timeout.\n");
            goto transError;
            }
        }
    else
        {

        /* polling mode*/

        pollTimeout  = SPI_FIFO_FULL_LEN * 10u * USEC_PER_SECOND;
        pollTimeout /= pDrvCtrl->clkFrequency;
        pollTimeout++;

        CSR_SETBIT_4 (pDrvCtrl, BCM2837_SPI_CS, BCM2837_SPI_CS_TA);
        while (pDrvCtrl->txLen)
            {
            pollCount = pollTimeout;
            if (pDrvCtrl->txLen >= SPI_FIFO_FULL_LEN)
                {
                bcm2837WriteFifoFree (pDrvCtrl, SPI_FIFO_FULL_LEN);
                }
            else
                {
                bcm2837WriteFifoFree (pDrvCtrl, pDrvCtrl->txLen);
                }

            while (pollCount--)
                {
                bcm2837ReadFifo (pDrvCtrl);
                if (bcm2837TransferDone (pDrvCtrl) == OK)
                    break;
                vxbUsDelay(1);
                }

            if (pollCount == 0)
                {
                DBG_MSG (DBG_ERR, "poll timeout.\n");
                goto transError;
                }

            bcm2837ReadFifo (pDrvCtrl);
            }

        if (pDrvCtrl->rxLen > 0)
            {
            goto transError;
            }
        bcm2837SpiReset (pDrvCtrl);
        }

    return OK;

transError:
    bcm2837SpiReset (pDrvCtrl);
    return ERROR;
    }

/*******************************************************************************
*
* bcm2837SpiTransfer - SPI transfer routine
*
* This routine is used to perform one transmission. It is the interface which
* can be called by SPI device driver to send and receive data via the SPI
* controller.
*
* RETURNS: OK, or ERROR if failed to transfer
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837SpiTransfer
    (
    VXB_DEV_ID      pDev,
    SPI_HARDWARE *  devInfo,
    SPI_TRANSFER *  pPkg
    )
    {
    BCM2837_SPI_DRVCTRL * pDrvCtrl;

    pDrvCtrl = (BCM2837_SPI_DRVCTRL *) vxbDevSoftcGet (pDev);
    if (pDrvCtrl == NULL)
        {
        DBG_MSG (DBG_ERR, "pDrvCtrl is NULL \n");
        return ERROR;
        }

    if (bcm2837SpiTransferConfig (pDev, devInfo) != OK)
        {
        DBG_MSG (DBG_ERR, "bcm2837SpiTransferConfig failed.\n");
        return ERROR;
        }

    if (IS_SPI_FULL_DUPLEX (devInfo->mode))
        {
        pDrvCtrl->rxSkipLen = 0;
        pDrvCtrl->txLen     = (pPkg->txLen > pPkg->rxLen) ? pPkg->txLen :
                                                            pPkg->rxLen;
        }
    else
        {
        pDrvCtrl->rxSkipLen = pPkg->txLen;
        pDrvCtrl->txLen     = pPkg->txLen + pPkg->rxLen;
        }

    if (pPkg->txLen == 0)
        {
        pDrvCtrl->txBuf  = NULL;
        }
    else
        {
        pDrvCtrl->txBuf  = pPkg->txBuf;
        }
    pDrvCtrl->rxBuf     = pPkg->rxBuf;
    pDrvCtrl->rxLen     = pPkg->rxLen;
    pDrvCtrl->txBufEnd  = pDrvCtrl->txBuf + pPkg->txLen;

    return bcm2837SpiTransferStart (pDev, devInfo);
    }
