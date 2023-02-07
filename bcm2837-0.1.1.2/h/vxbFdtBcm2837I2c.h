/* vxbFdtBcm2837I2c.h - Broadcom I2C hardware defintions */

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

#ifndef __INCvxbFdtBcm2837I2ch
#define __INCvxbFdtBcm2837I2ch

#ifdef __cplusplus
extern "C" {
#endif

/* defines */

#define BCM2837_I2C_ICLK_FREQ    (150000000)

/* register definitions for alternative functions of GPIO */

#define BCM2837_GPIO_FSEL0_OFFSET           0x00
#define BCM2837_GPIO_FSEL_REG(pin)          (BCM2837_GPIO_FSEL0_OFFSET +\
                                             (pin) / 10 * 4u)
#define BCM2837_GPIO_FSEL_SHIFT(pin)         (((pin) % 10) * 3)
#define BCM2837_GPIO_FSEL_MASK              (0x7u)

#define BCM2837_GPIO_FSEL_GPIO_IN           (0u)
#define BCM2837_GPIO_FSEL_GPIO_OUT          (1u)
#define BCM2837_GPIO_FSEL_ALT0              (4u)
#define BCM2837_GPIO_FSEL_ALT1              (5u)
#define BCM2837_GPIO_FSEL_ALT2              (6u)
#define BCM2837_GPIO_FSEL_ALT3              (7u)
#define BCM2837_GPIO_FSEL_ALT4              (3u)
#define BCM2837_GPIO_FSEL_ALT5              (2u)

#define BCM2837_GPIO_FSEL_COUNT             (8)

#define BCM2837_GPIO_PROPERTY_FOR_I2C_SDA   "sda-pin"
#define BCM2837_GPIO_PROPERTY_FOR_I2C_SCL   "scl-pin"

/* register definitions for I2C */

#define BCM2837_I2C_CONTROL_REG              0x0u  /* Control */
#define BCM2837_I2C_STATUS_REG               0x4u  /* Status */
#define BCM2837_I2C_DLEN_REG                 0x8u  /* Data Length */
#define BCM2837_I2C_SADDR_REG                0xCu  /* Slave Address */
#define BCM2837_I2C_DFIFO_REG                0x10u /* Data FIFO */
#define BCM2837_I2C_CLKDIV_REG               0x14u /* Clock Divider */
#define BCM2837_I2C_DATADLY_REG              0x18u /* Data Delay */
#define BCM2837_I2C_CLKT_REG                 0x1Cu /* Clock Stretch Timeout */

/* bit field for control register */

#define BCM2837_I2C_C_I2CEN         (0x1u << 15)   /* I2C Enable */
#define BCM2837_I2C_C_INTR          (0x1u << 10)   /* INTR Interrupt on RX */
#define BCM2837_I2C_C_INTT          (0x1u << 9)    /* INTT Interrupt on TX */
#define BCM2837_I2C_C_INTD          (0x1u << 8)    /* INTD Interrupt on DONE */
#define BCM2837_I2C_C_ST            (0x1u << 7)    /* ST Start Transfer */
#define BCM2837_I2C_C_CLEAR         (0x3u << 4)    /* CLEAR FIFO Clear */
#define BCM2837_I2C_C_READ          (0x1u << 0)    /* READ Read Transfer */

#define BCM2837_I2C_C_CLEAR_MASK    (0x0030u)      /* CLEAR FIFO Clear */

/* bit field for status register */

#define BCM2837_I2C_STATUS_LEN      (0x1u << 10)   /* use to sign the status
                                                      is error, is not a actual
                                                      bit in this register */
#define BCM2837_I2C_STATUS_CLKT     (0x1u << 9)    /* Clock Stretch Timeout */
#define BCM2837_I2C_STATUS_ERR      (0x1u << 8)    /* ERR ACK Error */
#define BCM2837_I2C_STATUS_RXF      (0x1u << 7)    /* FIFO Full */
#define BCM2837_I2C_STATUS_TXE      (0x1u << 6)    /* FIFO Empty */
#define BCM2837_I2C_STATUS_RXD      (0x1u << 5)    /* FIFO contains Data */
#define BCM2837_I2C_STATUS_TXD      (0x1u << 4)    /* FIFO can accept Data */
#define BCM2837_I2C_STATUS_RXR      (0x1u << 3)    /* FIFO needs Reading */
#define BCM2837_I2C_STATUS_TXW      (0x1u << 2)    /* FIFO needs Writing */
#define BCM2837_I2C_STATUS_DONE     (0x1u << 1)    /* Transfer Done */
#define BCM2837_I2C_STATUS_TA       (0x1u << 0)    /* Transfer Active */

#define BCM2837_DIV_ROUND_UP(iclk,busSpeed)                                 \
                             ((iclk) + (busSpeed) - 1) / (busSpeed)

#define BCM2837_I2C_FEDL_SHIFT      16
#define BCM2837_I2C_REDL_SHIFT      0

#define BCM2837_I2C_CDIV_MIN        0x0002u
#define BCM2837_I2C_CDIV_MAX        0xFFFEu

#define BCM2837_I2C_WRTIME_DEFAULT  0
/* assume the slowest transfer speed is 1/5 of busFreq */

#define SEM_TIMEOUT         10    /* max timeout for each byte transmit */

/* I2C controller read and write interface */

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#    define SWAP64 vxbSwap64
#else
#    define SWAP32
#    define SWAP64
#endif /* ARMBE8 */

#define BCM_I2C_REG_READ(pI2c, offset)                                      \
            SWAP32(vxbRead32 (pI2c->i2cHandle,                              \
                      (UINT32 *)((UINT8 *)(pI2c->i2cRegBase) + offset)))

#define BCM_I2C_REG_WRITE(pI2c, offset, value)                              \
            vxbWrite32 (pI2c->i2cHandle,                                    \
                        (UINT32 *)((UINT8 *)(pI2c->i2cRegBase) + offset),   \
                        SWAP32(value))

#define BCM_I2C_GPIO_REG_READ(pI2c, offset)                                 \
            SWAP32(vxbRead32 (pI2c->pGpioHandle,                            \
                      (UINT32 *)((UINT8 *)(pI2c->gpioRegBase) + offset)))

#define BCM_I2C_GPIO_REG_WRITE(pI2c, offset, value)                         \
            vxbWrite32 (pI2c->pGpioHandle,                                  \
                        (UINT32 *)((UINT8 *)(pI2c->gpioRegBase) + offset),  \
                        SWAP32(value))

#define BCM_I2C_REG_SETBIT_4(pI2c, offset, data)                            \
        vxbWrite32 (pI2c->i2cHandle,                                        \
                    (UINT32 *)((UINT8 *)(pI2c->i2cRegBase) + offset),       \
                    SWAP32 ((UINT32)(data) |                                \
                             BCM_I2C_REG_READ(pI2c, offset)))

#define BCM_I2C_REG_CLEARBIT_4(pI2c, offset, data)                          \
        vxbWrite32 (pI2c->i2cHandle,                                        \
                    (UINT32 *)((UINT8 *)(pI2c->i2cRegBase) + offset),       \
                    SWAP32 ((UINT32)(~(data)) &                             \
                             BCM_I2C_REG_READ(pI2c, offset)))

#define I2C_CNT_MAX              65535

/* structure holding the instance specific details */

typedef struct i2c_drv_ctrl {
    VXB_DEV_ID      i2cDev;
    void *          pGpioHandle;
    VIRT_ADDR       gpioRegBase;
    void *          i2cHandle;
    VIRT_ADDR       i2cRegBase;
    UINT32          i2cSdaPin;
    UINT32          i2cSclPin;
    SEM_ID          i2cDevSem;
    SEM_ID          i2cDataSem;
    UINT64          iClkFreq;
    UINT32          busSpeed;
    UINT32          defBusSpeed;
    BOOL            polling;
    BOOL            intEnabled;
    I2C_MSG *       msgs;
    int             numOfMsgs;
    UINT32          msgError;
    UINT8 *         dataBuf;
    UINT32          dataLength;
    UINT32          totalLen;
    UINT32          dataSAddr;
    VXB_RESOURCE *  intRes;
} BCM_I2C_DRV_CTRL;

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbFdtBcm2837I2ch */
