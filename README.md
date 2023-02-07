VxWorks® 7 Raspberry Pi 3B/3B+ Community BSP
===
---

# Overview

This document describes the features of the rpi_3 BSP/PSL, which is designed
to run on the Raspberry Pi 3 Model B/B+ board. This is an ARM Cortex-A53
processor-based platform.

# Project License

Copyright (c) 2019-2023 Wind River Systems, Inc.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1) Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3) Neither the name of Wind River Systems nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

# Prerequisite(s)

* You must have Wind River® VxWorks® 7 21.07 released source code and
  development environment to support "Raspberry Pi 3B/B+ Community BSP".

# Building and Using

### Preparation

IMPORTANT: Your existing VxWorks 7 installation may already contains the rpi_3 BSP 
and bcm2837 PSL, please check the following installation location to see whether the 
BSP and PSL existed already. 
The rpi_3 BSP is installed at:
```Bash
***installDir***/vxworks/21.07/os/unsupported/rpi_3/rpi_3
```
The bcm2837 PSL is installed at:
```Bash
***installDir***/vxworks/21.07/os/unsupported/rpi_3/bcm2837
```
If the installed version of BSP/PSL is the same or newer than the Open Source BSP 
published here, there's no need to download and install the Open Source BSP from 
GitHub again.

If your existing VxWorks 7 installation contains older version of rpi_3 BSP and 
bcm2837 PSL, it's recommended to download the latest code from GitHub and then 
install.

### Download

Download all layers from Github.
```Bash
git clone https://github.com/danieleftodi/vxw7-bsp-raspberry-pi.git
cd vxw7-bsp-raspberry-pi
```

### Installation

There are two ways to install this BSP: inside existing VxWorks 7 Installation or outside
existing VxWorks 7 Installation.

#### Install into the source tree

All layers in this BSP go to their respective destination among the existing installation. 
The advantage is the BSP will always be accessible once you complete the installation. The 
disadvantage is you can't shut down this BSP unless you manually delete all the installed 
layers among the source tree.

Here's how it’s done:

```Bash
cp -r rpi_3 ***installDir***/vxworks/21.07/os/unsupported/rpi_3/
cp -r bcm2837 ***installDir***/vxworks/21.07/os/unsupported/rpi_3/
```

#### Install beside the source tree

All layers are copied in a place that's outside the default scanning folder, i.e., 
vxworks/21.07/os/unsupported/rpi_3, and when launching the Development Shell or Workbench, the path containing 
this BSP is provided to the development environment. The advantage of this method is obvious, 
the package can be easily turn on or off, and the source code stays in one unified location 
external to the default installation, which makes it easier to manage.

Suppose all packages are in /home/rpi_3/, then do the following when launching wrenv
or Workbench:

```Bash
export WIND_LAYER_PATHS=/home/rpi_3
export WIND_BSP_PATHS=/home/rpi_3
```
then enter into the existing VxWorks 7 (21.07) installation directory
```Bash
cd ***installDir***
```
if use the Development Shell
```Bash
./wrenv.sh -p vxworks-7
```
or Workbench
```Bash
./workbench-4/startWorkbench.sh
```

### Building

The building process comprises three parts, U-Boot, VSB project and VIP project.
A Wind River-compiled U-boot image had been added into this BSP, you can use it 
directly. Alternatively, you can get the U-Boot code from related official server. 
The building processes of these three parts are all described in target.txt. target.txt 
can be found under rpi_3/ directory.

### Drivers

By now, the support of drivers in this Open Source BSP is as below:

| Hardware Interface | Controller | Driver / Component              | Status      |
| ------------------ | ---------- | ------------------------------- | ----------- |
| Mini UART          | on-chip    | DRV_SIO_FDT_NS16550             | SUPPORTED   |
| BCM2837 L1 INTCTLR | on-chip    | DRV_INTCTLR_FDT_BCM2837_L1_INTC | SUPPORTED   |
| BCM2837 INTCTLR    | on-chip    | DRV_INTCTLR_FDT_BCM2837_INTC    | SUPPORTED   |
| ARM Generic Timer  | on-chip    | DRV_ARM_GEN_TIMER               | SUPPORTED   |
| System Timer       | on-chip    | DRV_TIMER_FDT_BCM2837_SYSTIMER  | SUPPORTED   |
| Clock              | on-chip    | N/A                             | UNSUPPORTED |
| GPIO               | on-chip    | DRV_GPIO_FDT_BCM2837            | SUPPORTED   |
| I2C                | on-chip    | DRV_I2C_FDT_BCM2837             | SUPPORTED   |
| PinMux             | on-chip    | N/A                             | UNSUPPORTED |
| SPI                | on-chip    | DRV_SPI_FDT_BCM2837             | SUPPORTED   |
| USB HOST           | on-chip    | USB Host Stack(DWC2DR)          | SUPPORTED   |
| USB ETHERNET       | on-chip    | INCLUDE_USB_GEN2_LAN78XX        | SUPPORTED   |
| USB ETHERNET       | on-chip    | INCLUDE_USB_GEN2_LAN9514        | SUPPORTED   |
| Audio              | on-chip    | N/A                             | UNSUPPORTED |
| HDMI               | on-chip    | N/A                             | UNSUPPORTED |
| Watchdog Module    | on-chip    | N/A                             | UNSUPPORTED |
| EMMC               | on-chip    | N/A                             | UNSUPPORTED |
| Wireless LAN       | unknown    | N/A                             | UNSUPPORTED |
| Bluetooth          | unknown    | N/A                             | UNSUPPORTED |

The detailed introduction of these drivers and usage can also be found in target.txt.

# Legal Notices

All product names, logos, and brands are property of their respective owners. All company, product 
and service names used in this software are for identification purposes only. Wind River and VxWorks 
are registered trademarks of Wind River Systems. Raspberry Pi are registered trademarks of the 
Raspberry Pi Foundation.

Disclaimer of Warranty / No Support: Wind River does not provide support and maintenance services 
for this software, under Wind River’s standard Software Support and Maintenance Agreement or otherwise. 
Unless required by applicable law, Wind River provides the software (and each contributor provides its 
contribution) on an “AS IS” BASIS, WITHOUT WARRANTIES OF ANY KIND, either express or implied, including, 
without limitation, any warranties of TITLE, NONINFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A PARTICULAR 
PURPOSE. You are solely responsible for determining the appropriateness of using or redistributing the 
software and assume ay risks associated with your exercise of permissions under the license.

This BSP is not covered by Wind River's standard support. Should you need support, please work with 
the community, or should you need commercial support, please contact your local Wind River Sales Representative.
