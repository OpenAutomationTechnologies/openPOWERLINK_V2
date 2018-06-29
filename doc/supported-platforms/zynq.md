openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
================================

[TOC]

On Xilinx Zynq ZC702 SoC, openPOWERLINK can be run under Linux. This page will
give an overview of the supported environments and explains the steps to
build and run openPOWERLINK on the Xilinx Zynq SoC.

Currently, openPOWERLINK can be used under the following environments on a Zynq SoC:

# Software-only design - Zynq Emacps

In this design, openPOWERLINK is implemented on Linux which is running on the
ARM processing system (PS) of the SoC. This design uses the on-board MAC of
the Zynq ZC702 evaluation board.
Refer to the document [build and execute Zynq Emacps design](\ref page_zynq_emacps)
for more infomration.

# FPGA-supported design - Zynq Hybrid

In this design, the ARM processing system (PS) runs the openPOWERLINK user
layer along with the application. The time-critical kernel part of the stack
is implemented on a Microblaze softcore processor as a bare metal application
which is located in the programming logic (PL) of the Zynq SoC. For the network
connection, the openMAC IP-Core is used. The user and kernel layer exchange the
data and control information via a shared memory interface.

This design requires additional hardware, the AVNET expander board
(AES-FMC-ISMNET-G), which is connected to the FMC connector of the Zynq ZC702
evaluation board. This expander board adds two 1588 compatible 10/100
Ethernet PHYs as well as CAN, RS232 and RS485.
Refer the document to [build and execute Zynq Hybrid MN design](\ref page_mn_zynq_hybrid)
on openPOWERLINK.
Refer the document to [build and execute Zynq Hybrid CN design](\ref page_cn_zynq_hybrid)
on openPOWERLINK.
