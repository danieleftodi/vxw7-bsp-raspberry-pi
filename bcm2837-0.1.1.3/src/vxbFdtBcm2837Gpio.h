/* vxbFdtBcm2837Gpio.h - Bcm2837 GPIO controller driver header */

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

#ifndef	__INCvxbFdtNxpBcm2837Gpioh
#define	__INCvxbFdtNxpBcm2837Gpioh

#ifdef __cplusplus
extern "C" {
#endif

/* defines */

#define BCM2837_GPIO_SEL_MASK             0x7u
#define BCM2837_GPIO_INPUT_SEL            0x0u
#define BCM2837_GPIO_OUTPUT_SEL           0x1u
#define BCM2837_GPIO_BANK_WIDTH           54u
#define BCM2837_GPIO_EINT_RANGE           BCM2837_GPIO_BANK_WIDTH

/* GPIO register offsets */

/* for PinMux */

#define BCM2837_GPFSEL0         0x00u
#define BCM2837_GPPUD           0x94u
#define BCM2837_GPPUDCLK0       0x98u
#define BCM2837_GPPUDCLK1       0x9cu

/* for GPIO */

#define BCM2837_GPSET0          0x1cu
#define BCM2837_GPSET1          0x20u

#define BCM2837_GPCLR0          0x28u
#define BCM2837_GPCLR1          0x2cu

#define BCM2837_GPLEV0          0x34u
#define BCM2837_GPLEV1          0x38u

#define BCM2837_GPEDS0          0x40u
#define BCM2837_GPEDS1          0x44u

#define BCM2837_GPREN0          0x4cu
#define BCM2837_GPREN1          0x50u

#define BCM2837_GPFEN0          0x58u
#define BCM2837_GPFEN1          0x5cu

#define BCM2837_GPHEN0          0x64u
#define BCM2837_GPHEN1          0x68u

#define BCM2837_GPLEN0          0x70u
#define BCM2837_GPLEN1          0x74u

/* async(quick) edge detect, no need clock */

#define BCM2837_GPAREN0         0x7cu
#define BCM2837_GPAREN1         0x80u
#define BCM2837_GPAFEN0         0x88u
#define BCM2837_GPAFEN1         0x8cu

typedef enum
    {
    BCM2837_GPIO_PULL_DISABLED = 0,
    BCM2837_GPIO_PULL_DOWN,
    BCM2837_GPIO_PULL_UP,
    BCM2837_GPIO_PULL_RESERVED
    } BCM2837_GPIO_PULL_TYPE;

#ifdef __cplusplus
}
#endif

#endif	/* __INCvxbFdtNxpBcm2837Gpioh */
