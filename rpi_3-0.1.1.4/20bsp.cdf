/* 20bsp.cdf - BSP component description file */

/*
 * Copyright (c) 2019-2021 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
modification history
--------------------
02mar21,swu  update board names (VXWPG-1607)
23mar20,k_s  removed parameters NUM_TTY and CONSOLE_TTY (V7COR-7004)
08mar19,npc  created (F11409)
*/

/*
 * VX_SMP_NUM_CPUS is an SMP parameter and is ignored for UP.
 * VX_SMP_NUM_CPUS overrides the value defined in 00vxWorks.cdf.
*/

Parameter VX_SMP_NUM_CPUS {
    DEFAULT     (INCLUDE_RPI_3B_PLUS)::(4) \
                (1)
}

Selection BOARD_SELECTION {
    NAME        Board Selection
    COUNT       1-1
    CHILDREN    INCLUDE_RPI_3B_PLUS
    DEFAULTS    INCLUDE_RPI_3B_PLUS
    _CHILDREN   FOLDER_HIDDEN
}

Component INCLUDE_RPI_3B_PLUS {
    NAME        Raspberry Pi 3 Model B+
    SYNOPSIS    This component configures your project to support the \
                Raspberry Pi 3 Model B+.
}

Parameter DTS_FILE {
    NAME        Device tree source (DTS) file name
    SYNOPSIS    This parameter specifies the DTS file to be used.
    DEFAULT     (INCLUDE_RPI_3B_PLUS)::(rpi-3b-plus.dts)   \
                (rpi-3b-plus.dts)
}

Parameter LOCAL_MEM_PHYS_ADRS {
    NAME        Local memory physical base address
    SYNOPSIS    This parameter specifies the physical base address for memory.
    DEFAULT     0x0
}

Parameter RAM_LOW_ADRS {
    NAME        Runtime kernel entry address
    DEFAULT     0xffffffff80100000
}

Parameter RAM_HIGH_ADRS {
    NAME        Runtime kernel high address
    DEFAULT     0xffffffff81000000
}

Parameter LOCAL_MEM_LOCAL_ADRS {
    NAME        System memory start address
    SYNOPSIS    This parameter specifies the virtual base address for memory.
    DEFAULT     0xffffffff80000000
}

Parameter DTB_RELOC_ADDR {
    NAME        Device tree blob (DTB) relocation address
    SYNOPSIS    The DTB must be relocated to a safe address so that it is not \
                overwritten. This address should be below RAM_LOW_ADRS and the \
                reserved start region. This address must allow sufficient space \
                for the entire DTB.
    TYPE        void *
    DEFAULT     (LOCAL_MEM_LOCAL_ADRS + 0x10000)
}

Parameter DTB_MAX_LEN {
    NAME        DTB maximum length
    SYNOPSIS    DTB(Device Tree Blob) need to be relocated to one safe \
                address to avoid be overwritten, so it should be below \
                RAM_LOW_ADRS and the reserved start region, and enough \
                for the entire DTB.
    TYPE        int
    DEFAULT     0x20000
}

Parameter KERNEL_LOAD_ADRS {
    NAME        Runtime kernel load address
    DEFAULT     0x00100000
}

Parameter IMA_SIZE {
    NAME        Initial mapped area (IMA) size
    SYNOPSIS    IMA is mapped to the MMU during the early initialization phase \
                (before usrMmuInit()). Therefore, the size must be large enough \
                to accommodate the entire VxWorks kernel image.
    DEFAULT     0x10000000
}

Parameter ISR_STACK_SIZE {
    NAME        ISR stack size
    SYNOPSIS    This parameter defines the stack size in bytes for the \
                interrupt service routine (ISR).
    DEFAULT     0x2000
}

Parameter DEFAULT_BOOT_LINE {
    NAME        Default boot line
    SYNOPSIS    This parameter provides the default boot line string.
    TYPE        string
    DEFAULT     "dummy(0,0)host:vxWorks h=192.168.0.2 e=192.168.0.3 u=target pw=vxTarget"
}

Parameter CONSOLE_BAUD_RATE {
    NAME        Baud rate of console port
    SYNOPSIS    This parameter specifies the baud rate of the console serial \
                port.
    DEFAULT     115200
}

Profile BSP_DEFAULT {
    PROFILES    PROFILE_OS_DEFAULT
    COMPONENTS  += DRV_SIO_FDT_NS16550
}

Component DRV_SIO_FDT_NS16550 {
    INCLUDE_WHEN    INCLUDE_SIO
}

Component DRV_ARM_GEN_TIMER {
    INCLUDE_WHEN    INCLUDE_TIMER_SYS
}

Component DRV_TIMER_FDT_BCM2837_SYSTIMER {
    INCLUDE_WHEN    INCLUDE_AUX_CLK
}

Parameter SYSCLK_TIMER_NAME {
    NAME        System clock device name
    SYNOPSIS    This parameter sets the system clock device. If the value is \
                NULL, the device is auto-selected.
    TYPE        string
    DEFAULT     "armGenTimer"
}

Parameter SYSCLK_TIMER_NUM {
    NAME        System Clock Timer Number
    TYPE        int
    DEFAULT     0
}

Parameter AUXCLK_TIMER_NAME {
    NAME        Auxiliary clock device name
    SYNOPSIS    This parameter sets the auxiliary clock device. If the value is \
                NULL, the device is auto-selected.
    TYPE        string
    DEFAULT     "bcm2837-system-timer"
}

Parameter AUXCLK_TIMER_NUM {
    NAME        Auxiliary Clock Timer Number
    TYPE        int
    DEFAULT     0
}
