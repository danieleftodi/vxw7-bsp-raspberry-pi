/* usb2Lan9514_rpi_3.c - LAN9514 device class driver using the WRS USB2 API */

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
DESCRIPTION

This module works as a hardware configure layer for USB-Ethernet adapters
built with Lan9514 Chipsets.

Since the module use the interface offered by usb2End.c, we only need to add
the supported device list into the the global device list g_usb2EndAdapterList
defined in usb2End.c. In the device list information structure, the
pPrivateFuncs table is very important, such as the hardware configuration
usb2Lan9514HWConfig.

\cs
USB2_END_PRIVATE_FUNCS usb2Lan9514PrivteFuncs =
    {
    usb2Lan9514SetFlag,
    NULL,
    usb2Lan9514HWConfig,
    usb2Lan9514StatusCheck,
    usb2Lan9514DataPacking,
    usb2Lan9514DataUnPacking,
    NULL,
    NULL
    };
\ce

INITIALIZATION

To initialize the Lan9514 devices support, usb2Lan9514Init needs to be called to
add the supported device list to the global device list g_usb2EndAdapterList
defined in usb2End.c

Furthermore, usb2Lan9514Init() needs to be called before the usb2EndInit(),
which will guarantee the device can be recognized if the devices have already
been connected when booting up.
*/

/* includes */

#include <vxWorks.h>
#include <usb2End.h>
#include <vxFdtLib.h>

/* defines */

#define USB2_LAN9514_REG_WRITE_CMD  0xA0u
#define USB2_LAN9514_REG_READ_CMD   0xA1u

#define RX_STATUS_FRAME_LENGTH(status) ((status >> 16) & 0x3fffu)

#define TX_COMMAND_A_FIRST_SEGMENT  0x2000u
#define TX_COMMAND_A_LAST_SEGMENT   0x1000u
#define TX_COMMAND_A_BUFSIZE_MASK   0x07ffu
#define TX_COMMAND_B_FRAMELEN_MASK  0x07ffu

#define TX_CFG      0x010u
#define HW_CFG      0x014u
#define PMT_CTL     0x020u
#define MAC_CR      0x100u
#define ADDRH       0x104u
#define ADDRL       0x108u
#define MII_ACCESS  0x114u
#define MII_DATA    0x118u

#define TX_CFG_TX_ON        0x00000004u
#define HW_CFG_BIR          0x00001000u
#define HW_CFG_LRST         0x00000008u
#define PMT_CTL_PHY_RST     0x00000010u
#define PMT_CTL_WRITE_MASK  0x0000037Cu
#define MAC_CR_MCPAS        0x00080000u
#define MAC_CR_PROMISC      0x00040000u
#define MAC_CR_PADSTR       0x00000100u
#define MAC_CR_TXEN         0x00000008u
#define MAC_CR_RXEN         0x00000004u
#define MII_ACCESS_MIIWNR   0x00000002u
#define MII_ACCESS_MIIBZY   0x00000001u
#define MII_ACCESS_PHY      0x00000800u

#define MII_ACCESS_MIIRINDA(idx) ((idx & 0x1fu) << 6)

#define PHYREG_BASIC_STATUS 1u
#define PHY_BSR_LINK_STATUS 0x0004u

/* imports */

IMPORT void sysUsDelay (int delay);

/* locals */

struct txCmd {
    UINT32 A;
    UINT32 B;
};

typedef UINT8 macaddr_t[6];
struct addrRegs {
    UINT32 addrl;
    UINT32 addrh;
};

LOCAL macaddr_t macaddr_debug = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 };

/* forward declarations */

LOCAL STATUS usb2Lan9514SetFlag     (END_OBJ *endObj);
LOCAL STATUS usb2Lan9514HWConfig    (VOID *pDev);
LOCAL VOID   usb2Lan9514StatusCheck (VOID *pDev);

LOCAL VOID   usb2Lan9514DataPacking
    (
    UINT8         * pBuf,
    UINT32        * pActLen
    );

LOCAL UINT32 usb2Lan9514DataUnPacking
    (
    unsigned char ** ppBuf,
    UINT32         * pPacketLen,
    UINT32         * pActLen
    );

/* Lan9514 device private function table */

LOCAL USB2_END_PRIVATE_FUNCS usb2Lan9514PrivteFuncs =
    {
    usb2Lan9514SetFlag,
    NULL,
    usb2Lan9514HWConfig,
    usb2Lan9514StatusCheck,
    usb2Lan9514DataPacking,
    usb2Lan9514DataUnPacking,
    NULL,
    NULL
    };

/* Configuration */

LOCAL USB2_END_CONFIG_FLAG usb2Lan9514ConfigFlag =
    {
    0,                      /* ID for drivers that support multiple chips */
    sizeof(struct txCmd),   /* bytes to allocate before an Ethernet frame */
    0,                      /* offset of link status in an Interrupt packet */
    0                       /* bit mask of link status in Interrupt packet */
    };

/* Adapter list */

LOCAL USB2_END_ADAPTER_INFO Usb2Lan9514AdapterList[] =
    {
    USB2_END_ADAPTER (0x0424,
                      0xec00,
                      "LAN9514 USB Ethernet Controller",
                      &usb2Lan9514PrivteFuncs,
                      &usb2Lan9514ConfigFlag
                      )
    };

/* usb2Lan9514DecodeMac - translate struct addrRegs to macadddr_t */

#if 0
LOCAL VOID usb2Lan9514DecodeMac
    (
    const struct addrRegs * regs,
    macaddr_t             * addr
    )
    {
    (*addr)[0] = (UINT8) (regs->addrl      ) & 0xff;
    (*addr)[1] = (UINT8) (regs->addrl >>  8) & 0xff;
    (*addr)[2] = (UINT8) (regs->addrl >> 16) & 0xff;
    (*addr)[3] = (UINT8) (regs->addrl >> 24) & 0xff;
    (*addr)[4] = (UINT8) (regs->addrh      ) & 0xff;
    (*addr)[5] = (UINT8) (regs->addrh >>  8) & 0xff;
    }
#endif

/* usb2Lan9514EncodeMac - translate macadddr_t to struct addrRegs */

LOCAL VOID usb2Lan9514EncodeMac
    (
    const macaddr_t * addr,
    struct addrRegs * regs
    )
    {
    regs->addrl = ((UINT32) (*addr)[0]      )
                | ((UINT32) (*addr)[1] <<  8)
                | ((UINT32) (*addr)[2] << 16)
                | ((UINT32) (*addr)[3] << 24);
    regs->addrh = ((UINT32) (*addr)[4]      )
                | ((UINT32) (*addr)[5] <<  8);
    }

/*******************************************************************************
*
* usb2Lan9514ReadReg - read a value from the Control and Status Registers (CSR)
*
* RETURNS: OK if the command succeeds, ERROR if it does not
*/

LOCAL STATUS usb2Lan9514ReadReg
    (
    USB2_CLASS_DEVICE * pUsbDev,
    UINT16              addr,               /* 12-bit register address */
    UINT32            * pData               /* All reads return 32 bits */
    )
    {
    USBHST_STATUS       hstStatus = USBHST_FAILURE;
    UINT16              length = sizeof(*pData);

    hstStatus = usb2VendorClassSpecific (
                        pUsbDev,
                        USB2_RT_DEV_TO_HOST | USB2_RT_VENDOR | USB2_RT_DEVICE,
                        USB2_LAN9514_REG_READ_CMD,
                        0,
                        addr,
                        &length,
                        (pUINT8) pData,
                        USB2_DEFAULT_TIMEOUT);

    USB2_END_VDBG ("%s(): CSR[0x%X] = 0x%X\n",
                   __FUNCTION__, addr, *pData, 4, 5, 6);

    return hstStatus == USBHST_SUCCESS ? OK : ERROR;
    }

/*******************************************************************************
*
* usb2Lan9514WriteReg - write a value to the Control and Status Registers (CSR)
*
* RETURNS: OK if the command succeeds, ERROR if it does not
*/

LOCAL STATUS usb2Lan9514WriteReg
    (
    USB2_CLASS_DEVICE * pUsbDev,
    UINT16              addr,               /* 12-bit register address */
    UINT32              data                /* All writes are 32 bits */
    )
    {
    USBHST_STATUS       hstStatus = USBHST_FAILURE;
    UINT16              length = sizeof(data);

    USB2_END_VDBG ("%s(): 0x%X => CSR[0x%X]\n",
                   __FUNCTION__, data, addr, 4, 5, 6);

    hstStatus = usb2VendorClassSpecific (
                        pUsbDev,
                        USB2_RT_HOST_TO_DEV | USB2_RT_VENDOR | USB2_RT_DEVICE,
                        USB2_LAN9514_REG_WRITE_CMD,
                        0,
                        addr,
                        &length,
                        (pUINT8) &data,
                        USB2_DEFAULT_TIMEOUT);

    return hstStatus == USBHST_SUCCESS ? OK : ERROR;
    }

/*******************************************************************************
*
* usb2Lan9514WaitRegisterBit - wait for a specific value in a register
*
* Read the given register repeatedly until the masked bits match
* the expected value, or timeout milliseconds have elapsed.
*
* RETURNS: OK once the condition is met, ERROR on timeout or another failure
*/

LOCAL STATUS usb2Lan9514WaitRegisterBit
    (
    USB2_CLASS_DEVICE * pUsbDev,
    UINT16              addr,
    UINT32              bitMask,
    UINT32              expected,
    UINT32              timeout             /* Timeout time in millisecond */
    )
    {
    UINT32              count = 0;
    UINT32              data;

#define READS_PER_MS 20
#define DELAY_US (1000 / READS_PER_MS)

    timeout *= READS_PER_MS;
    while (count < timeout)
        {
        if (OK != usb2Lan9514ReadReg (pUsbDev, addr, &data))
            {
            USB2_END_ERR ("%s(): failed to read register 0x%X\n",
                          __FUNCTION__, addr, 3, 4, 5, 6);
            return ERROR;
            }

        if ((data & bitMask) == expected)
            return OK;

        count++;
        sysUsDelay (DELAY_US);
        }

#undef DELAY_US
#undef READS_PER_MS

    return ERROR;   /* timeout */
    }

LOCAL STATUS usb2Lan9514WaitMiiBusy
    (
    USB2_CLASS_DEVICE * pUsbDev
    )
    {
    return usb2Lan9514WaitRegisterBit (pUsbDev,
                                       MII_ACCESS,
                                       MII_ACCESS_MIIBZY,
                                       0,
                                       1000);
    }

/*******************************************************************************
*
* usb2Lan9514PhyRead - read a value from the PHY Registers
*
* RETURNS: OK if the command succeeds, ERROR if it does not
*/

LOCAL STATUS usb2Lan9514PhyRead
    (
    USB2_CLASS_DEVICE * pUsbDev,
    UINT8               index,
    UINT16            * pPhyData
    )
    {
    UINT32 data;
    if (OK != usb2Lan9514WaitMiiBusy (pUsbDev)
    ||  OK != usb2Lan9514WriteReg (pUsbDev,
                                   MII_ACCESS,
                                   MII_ACCESS_PHY
                                 | MII_ACCESS_MIIRINDA(index)
                                 | MII_ACCESS_MIIBZY)
    ||  OK != usb2Lan9514WaitMiiBusy (pUsbDev)
    ||  OK != usb2Lan9514ReadReg (pUsbDev, MII_DATA, &data))
        return ERROR;

    *pPhyData = data & 0xFFFFu;
    return OK;
    }

/*******************************************************************************
*
* usb2Lan9514PhyWrite - write a value to the PHY Registers
*
* RETURNS: OK if the command succeeds, ERROR if it does not
*/

#if 0
LOCAL STATUS usb2Lan9514PhyWrite
    (
    USB2_CLASS_DEVICE * pUsbDev,
    UINT8               index,
    UINT16              phyData
    )
    {
    UINT32 data = MII_ACCESS_PHY
                | MII_ACCESS_MIIRINDA(index)
                | MII_ACCESS_MIIWNR
                | MII_ACCESS_MIIBZY;

    if (OK == usb2Lan9514WaitMiiBusy (pUsbDev)
    &&  OK == usb2Lan9514WriteReg (pUsbDev, MII_DATA, phyData)
    &&  OK == usb2Lan9514WriteReg (pUsbDev, MII_ACCESS, data)
    &&  OK == usb2Lan9514WaitMiiBusy (pUsbDev))
        return OK;
    else
        return ERROR;
    }
#endif

/*******************************************************************************
*
* usb2Lan9514ReadMac - retrieve the MAC address from the Ethernet Controller
*
* RETURNS: OK if the address is successfully retrieved, ERROR if not
*/

# if 0
LOCAL STATUS usb2Lan9514ReadMac
    (
    USB2_CLASS_DEVICE * pUsbDev,
    macaddr_t         * pAddr
    )
    {
    struct addrRegs data;

    if (OK != usb2Lan9514WriteReg (pUsbDev, ADDRL, data.addrl)
    ||  OK != usb2Lan9514WriteReg (pUsbDev, ADDRH, data.addrh))
        return ERROR;

    usb2Lan9514DecodeMac(&data, pAddr);
    return OK;
    }
#endif

/*******************************************************************************
*
* usb2Lan9514WriteMac - assign an address to the Media Access Controller
*
* RETURNS: OK if the address is successfully written, ERROR if not
*/

LOCAL STATUS usb2Lan9514WriteMac
    (
    USB2_CLASS_DEVICE * pUsbDev,
    const macaddr_t   * pAddr
    )
    {
    struct addrRegs data;
    usb2Lan9514EncodeMac(pAddr, &data);

    if (OK == usb2Lan9514WriteReg (pUsbDev, ADDRL, data.addrl)
    &&  OK == usb2Lan9514WriteReg (pUsbDev, ADDRH, data.addrh))
        return OK;
    else
        return ERROR;
    }

/*******************************************************************************
*
* usb2Lan9514ReadFdtMac - read the MAC address from the device tree
*
* RETURNS: OK if the address is successfully retrieved, ERROR if not
*/

LOCAL STATUS usb2Lan9514ReadFdtMac (macaddr_t *pAddr)
    {
    INT32           offset;
    INT32           valueLen;
    const void    * pValue = NULL;
    char          * compatible = "usb424,ec00";
    char          * property = "local-mac-address";

    offset = vxFdtNodeOffsetByCompatible (0, compatible);

    if (offset < 0)
        {
        USB2_END_ERR ("%s: no node is compatible with \"%s\"\n",
                      __FUNCTION__, compatible, 3, 4, 5, 6);
        return ERROR;
        }

    pValue = vxFdtPropGet (offset, property, &valueLen);

    if (pValue == NULL || (UINT32) valueLen < sizeof(macaddr_t))
        {
        USB2_END_ERR ("%s: node does not define \"%s\"\n",
                      __FUNCTION__, property, 3, 4, 5, 6);
        return ERROR;
        }

    memcpy (pAddr, pValue, sizeof(macaddr_t));
    USB2_END_INFO ("usb2Lan9514ReadFdtMac(): "
                   "%02x:%02x:%02x:%02x:%02x:%02x\n",
                   (*pAddr)[0], (*pAddr)[1], (*pAddr)[2],
                   (*pAddr)[3], (*pAddr)[4], (*pAddr)[5]);

    return OK;
    }

/*******************************************************************************
*
* usb2Lan9514StatusCheck - check Ethernet link status
*
* The usb2End driver will periodically call this function to determine
* whether the Ethernet link is up or down
*
* RETURNS: N/A
*/

LOCAL VOID usb2Lan9514StatusCheck (VOID *pDev)
    {
    USB2_END_DEVICE   * pDevice = NULL;
    USB2_CLASS_DEVICE * pUsbDev = NULL;
    UINT16              phyData = 0;

    pDevice = (USB2_END_DEVICE *) pDev;

    if (pDevice == NULL)
        return;

    pUsbDev = pDevice->pUsb2ClassDevice;

    if (pUsbDev == NULL)
        return;

    if (OK != usb2Lan9514PhyRead (pUsbDev, PHYREG_BASIC_STATUS, &phyData))
        {
        USB2_END_ERR ("%s(): Failed to read PHY Basic Status Register",
                      __FUNCTION__, 2, 3, 4, 5, 6);
        return;
        }

    BOOL linkIsUp = (phyData & PHY_BSR_LINK_STATUS) ? TRUE : FALSE;
    (void) usb2EndLinkUpdate(pDevice, linkIsUp);
    }

/*******************************************************************************
*
* usb2Lan9514LiteReset - perform a Soft Lite Reset of the Ethernet controller
*
* RETURNS: OK once the reset is done, ERROR if something goes wrong
*/

LOCAL STATUS usb2Lan9514LiteReset (USB2_CLASS_DEVICE *pUsbDev)
    {
    if (OK == usb2Lan9514WriteReg       (pUsbDev, HW_CFG, HW_CFG_LRST)
    &&  OK == usb2Lan9514WaitRegisterBit(pUsbDev, HW_CFG, HW_CFG_LRST, 0, 1000))
        return OK;
    else
        return ERROR;
    }

/*******************************************************************************
*
* usb2Lan9514PhyReset - power cycle the Ethernet PHY
*
* RETURNS: OK once the reset is done, ERROR if something goes wrong
*/

LOCAL STATUS usb2Lan9514PhyReset (USB2_CLASS_DEVICE *pUsbDev)
    {
    UINT32 data;
    if (OK == usb2Lan9514ReadReg (pUsbDev, PMT_CTL, &data)
    &&  OK == usb2Lan9514WriteReg(pUsbDev,
                                  PMT_CTL,
                                  (data | PMT_CTL_PHY_RST) & PMT_CTL_WRITE_MASK)
    &&  OK == usb2Lan9514WaitRegisterBit(pUsbDev,
                                         PMT_CTL,
                                         PMT_CTL_PHY_RST,
                                         0,
                                         1000))
        return OK;
    else
        return ERROR;
    }

/*******************************************************************************
*
* usb2Lan9514HWConfig - initialize and configure the Ethernet controller
*
* RETURNS: OK if configuration is successful, ERROR if something goes wrong
*/

LOCAL STATUS usb2Lan9514HWConfig (VOID *pDev)
    {
    USB2_END_DEVICE   * pDevice = NULL;
    USB2_CLASS_DEVICE * pUsbDev = NULL;

    pDevice = (USB2_END_DEVICE *) pDev;

    if (pDevice == NULL)
        return ERROR;

    pUsbDev = pDevice->pUsb2ClassDevice;

    if (pUsbDev == NULL)
        return ERROR;

    if (OK != usb2Lan9514ReadFdtMac (&pDevice->macAddress))
        {
        USB2_END_ERR ("%s(): Falling back to debug MAC address\n",
                      __FUNCTION__, 2, 3, 4, 5, 6);

        memcpy(&pDevice->macAddress, &macaddr_debug, sizeof(macaddr_t));
        }

    /* reset the Ethernet controller */

    if (OK != usb2Lan9514LiteReset(pUsbDev))
        {
        USB2_END_ERR("%s(): software lite reset failed\n",
                     __FUNCTION__, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* reset the Ethernet PHY */

    if (OK != usb2Lan9514PhyReset (pUsbDev))
        {
        USB2_END_ERR ("%s(): PHY reset failed\n",
                      __FUNCTION__, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* configure UDC to return NAK when no Bulk-In data is available */

    if (OK != usb2Lan9514WriteReg (pUsbDev, HW_CFG, HW_CFG_BIR))
        {
        USB2_END_ERR ("%s(): failed to set Bulk-In Empty Response\n",
                      __FUNCTION__, 2, 3, 4, 5, 6);
        /* not a fatal error */
        }

    /* configure the MAC address */

    if (OK != usb2Lan9514WriteMac (pUsbDev, &pDevice->macAddress))
        {
        USB2_END_ERR ("%s(): failed to set MAC address\n",
                      __FUNCTION__, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Enable receiver and transmitter */

    if (OK != usb2Lan9514WriteReg (pUsbDev, MAC_CR, MAC_CR_TXEN | MAC_CR_RXEN))
        {
        USB2_END_ERR ("%s(): failed to configure the MAC hardware layer\n",
                      __FUNCTION__, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Enable the transmit side of the FIFO Controller */

    if (OK != usb2Lan9514WriteReg (pUsbDev, TX_CFG, TX_CFG_TX_ON))
        {
        USB2_END_ERR ("%s(): failed to enable the TX FIFO\n",
                      __FUNCTION__, 2, 3, 4, 5, 6);
        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Lan9514DataPacking - prepare an Ethernet frame to be sent by USB
*
* The usb2End driver calls this function to prepare an Ethernet frame for a
* Bulk Out transfer. It uses a single TX Command structure for the entire
* frame, so the first segment and last segment bits are always set, and the
* Buffer Size and Frame Length fields always match.
*
* RETURNS: N/A
*/

LOCAL VOID usb2Lan9514DataPacking
    (
    UINT8      *pBuf,
    UINT32     *pActLen
    )
    {
    struct txCmd *pCmd = (struct txCmd *) pBuf;

    pCmd->A = TX_COMMAND_A_FIRST_SEGMENT
            | TX_COMMAND_A_LAST_SEGMENT
            | (*pActLen & TX_COMMAND_A_BUFSIZE_MASK);
    pCmd->B = *pActLen & TX_COMMAND_B_FRAMELEN_MASK;

    *pActLen += sizeof(*pCmd);
    }


/*******************************************************************************
*
* usb2Lan9514DataUnPacking - decode data received from the Bulk-In endpoint
*
* The usb2End driver calls this function after each Bulk In transfer
* to split up the received data into Ethernet frames
*
* RETURNS: The PAD data length.
*/

LOCAL UINT32 usb2Lan9514DataUnPacking
    (
    unsigned char **ppBuf,
    UINT32 *pPacketLen,
    UINT32 *pActLen
    )
    {
    UINT32 *pRxStatus   = (UINT32 *) *ppBuf;
    UINT32  rxStatus    = *pRxStatus;
    UINT32  length      = RX_STATUS_FRAME_LENGTH(rxStatus);

    /* each frame is guaranteed to have 4-byte alignment */
    UINT32 uPadLen  = sizeof(UINT32) - (length % sizeof(UINT32));
    UINT32 uRemain  = *pPacketLen - sizeof(rxStatus);

    if (length > uRemain)
        length = uRemain;

    uRemain -= length;

    if (uPadLen > uRemain)
        uPadLen = uRemain;

    uRemain -= uPadLen;

    *ppBuf = (pUINT8) (pRxStatus + 1);
    *pPacketLen = uRemain;
    *pActLen = length;
    return uPadLen;
    }

/*******************************************************************************
*
* usb2Lan9514SetFlag - modify interface configuration
*
* This function is called in response to an ioctl() call using EIOCSFLAGS
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*/

LOCAL STATUS usb2Lan9514SetFlag
    (
    END_OBJ *           endObj
    )
    {
    USB2_END_DEVICE *   pDevice = NULL;
    UINT32              disable = 0;
    UINT32              enable = 0;
    UINT32              data;

    if (endObj == NULL)
        {
        return ERROR;
        }

    pDevice = (USB2_END_DEVICE *) endObj;

    /* enable/disable promiscuous mode */

    if (END_FLAGS_GET (endObj) & IFF_PROMISC)
        enable |= MAC_CR_PROMISC;
    else
        disable |= MAC_CR_PROMISC;

    /* enable/disable incoming multicast traffic */

    if ((END_FLAGS_GET (endObj) & IFF_MULTICAST)
    ||  (END_FLAGS_GET (endObj) & IFF_ALLMULTI))
        enable |= MAC_CR_MCPAS;
    else
        disable |= MAC_CR_MCPAS;

    /* strip Ethernet frame padding */

    if (END_FLAGS_GET (endObj) & IFF_NOTRAILERS)
        enable |= MAC_CR_PADSTR;
    else
        disable |= MAC_CR_PADSTR;

    /* read the current settings */

    if (OK != usb2Lan9514ReadReg (pDevice->pUsb2ClassDevice, MAC_CR, &data))
        return ERROR;

    if ((data & enable) == enable && (data & disable) == 0)
        {
        return OK;
        }
    else
        {
        data &= ~disable;
        data |= enable;

        return usb2Lan9514WriteReg (pDevice->pUsb2ClassDevice, MAC_CR, data);
        }
    }

/*******************************************************************************
*
* usb2Lan9514Init - register this driver with the usb2End driver
*
* RETURNS: OK, or ERROR if the the driver could not be initialized.
*/

STATUS usb2Lan9514Init (char *pName)
    {
    UINT16 arrayLen;

    if (pName == NULL)
        pName = "usb2Lan9514";

    arrayLen = sizeof(Usb2Lan9514AdapterList)/sizeof(Usb2Lan9514AdapterList[0]);
    return usb2EndDeviceListAdd (pName, Usb2Lan9514AdapterList, arrayLen);
    }

/*******************************************************************************
*
* usb2Lan9514DeInit - unregister this driver from the usb2End driver
*
* RETURNS: N/A
*/

void usb2Lan9514DeInit (void)
    {

    usb2EndDeviceListRemove (Usb2Lan9514AdapterList);
    }

