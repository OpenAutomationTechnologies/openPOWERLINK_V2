###################################-*-asm*-
#
# Copyright (c) 2001 Xilinx, Inc.  All rights reserved.
#
# Xilinx, Inc.
#
# XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
# COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
# ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
# STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
# IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
# FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
# XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
# THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
# ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
# FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS FOR A PARTICULAR PURPOSE.
#
# crt4.s
#
# C run-time for MicroBlaze applications that do not want vectors (compiled
# with -xl-mode-novectors)
# $Id: crt3.s,v 1.1.2.1 2009/06/16 21:29:06 vasanth Exp $
#
#######################################

/*
        MicroBlaze Vector Map for vector-less ELF images

         Address        Vector type                 Label
         -------        -----------                 ------

        # 0x00 #        (-- IMM --)
        # 0x04 #        Reset                       (-- Don't Care --)

        # 0x08 #        (-- IMM --)
        # 0x0c #        Software Exception          (-- Don't Care --)

        # 0x10 #        (-- IMM --)
        # 0x14 #        Hardware Interrupt          (-- Don't Care --)

        # 0x18 #        (-- IMM --)
        # 0x1C #        Breakpoint Exception        (-- Don't Care --)

        # 0x20 #        (-- IMM --)
        # 0x24 #        Hardware Exception          (-- Don't Care --)
*/


        .section .text
        .globl _start
        .align 2
        .ent _start
        .type _start, @function
_start:
    la    r13, r0, _SDA_BASE_             /* Set the Small Data Anchors and the stack pointer */
    la    r2, r0, _SDA2_BASE_
    la    r1, r0, _stack-16               /* 16 bytes (4 words are needed by crtinit for args and link reg */

    brlid    r15, _crtinit               /* Initialize BSS and run program */
    nop

    brlid   r15, exit                   /* Call exit with the return value of main */
    addik   r5, r3, 0

    /* Control does not reach here */
    .end _start


/*
        _exit
        Our simple _exit
*/
        .globl _exit
        .align 2
        .ent _exit
        .type _exit, @function
_exit:
    bri     0
    .end _exit
