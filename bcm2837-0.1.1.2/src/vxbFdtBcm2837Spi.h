/* vxbFdtBcm2837Spi.h - Bcm2837 SPI controller driver header */

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
08mar19,zxw  created (F11409)
*/

#ifndef __INCvxbFdtBcm2837Spih
#define __INCvxbFdtBcm2837Spih

#ifdef __cplusplus
extern "C"
{
#endif

/* defines */

#define MAX_DIVIDERATIO	    65536u

/* SPI built-in FIFO length, RXR is 3/4 of full length */

#define SPI_FIFO_FULL_LEN      64u
#define SPI_FIFO_RXR_LEN       48u
#define SPI_GPIO_BITS_MASK     7u
#define SPI_CS_ACTIVE_MASK     3u
#define SPI_CE0_BIT_OFFSET     24u
#define SPI_CE1_BIT_OFFSET     21u
#define SPI_MISO_BIT_OFFSET    27u
#define SPI_MOSI_BIT_OFFSET    0u
#define SPI_SCLK_BIT_OFFSET    3u
#define GPIO7_CLK_BIT_OFFSET   7u
#define GPIO8_CLK_BIT_OFFSET   8u
#define GPIO9_CLK_BIT_OFFSET   9u
#define GPIO10_CLK_BIT_OFFSET  10u
#define GPIO11_CLK_BIT_OFFSET  11u
#define SPI_PINMUX_ALT0        0x4u
#define USEC_PER_SECOND        1000000u

#define BCM2837_SPI_CS_CSPOL0   (1u << 21)
#define BCM2837_SPI_CS_RXF      (1u << 20)
#define BCM2837_SPI_CS_RXR      (1u << 19)
#define BCM2837_SPI_CS_TXD      (1u << 18)
#define BCM2837_SPI_CS_RXD      (1u << 17)
#define BCM2837_SPI_CS_DONE     (1u << 16)
#define BCM2837_SPI_CS_REN      (1u << 13)
#define BCM2837_SPI_CS_INTR     (1u << 10)
#define BCM2837_SPI_CS_INTD     (1u << 9)
#define BCM2837_SPI_CS_DMAEN    (1u << 8)
#define BCM2837_SPI_CS_TA       (1u << 7)
#define BCM2837_SPI_CS_CLEAR_RX (1u << 5)
#define BCM2837_SPI_CS_CLEAR_TX (1u << 4)
#define BCM2837_SPI_CS_CPOL     (1u << 3)
#define BCM2837_SPI_CS_CPHA     (1u << 2)
#define BCM2837_SPI_CS_CS1      (1u << 1)
#define BCM2837_SPI_CS_CS0      (1u << 0)

typedef enum
    {
    BCM2837_GPIO_PULL_DISABLED = 0,
    BCM2837_GPIO_PULL_DOWN,
    BCM2837_GPIO_PULL_UP,
    BCM2837_GPIO_PULL_RESERVED
    } BCM2837_GPIO_PULL_TYPE;

/* Register Addresses */

/* PinMux */

#define BCM2837_GPFSEL0         0x00u
#define BCM2837_GPPUD           0x94u
#define BCM2837_GPPUDCLK0       0x98u

/* SPI register */

#define BCM2837_SPI_CS      0x0u
#define BCM2837_SPI_FIFO    0x4u
#define BCM2837_SPI_CLK     0x8u
#define BCM2837_SPI_DLEN    0xcu
#define BCM2837_SPI_LTOH    0x10u
#define BCM2837_SPI_DC      0x14u

/* Number of chip selects supported */

#define SPI_MAX_CS_NUM             2u

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbFdtBcm2837Spih */
