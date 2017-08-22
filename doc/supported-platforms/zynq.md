openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
================================

[TOC]

On Xilinx Zynq ZC702 SoC, openPOWERLINK can be running under Linux. This page will
give an overview of the supported environments and explains the steps to
build and run openPOWERLINK on Zynq SoC.

Currently, openPOWERLINK can run under the following environments on a Zynq SoC:

# Single processor design - Zynq Emacps

openPOWERLINK runs on Linux which is running on the ARM processing system (PS)
of the SoC processor. This design uses the on-board MAC available on the
Zynq ZC702 evaluation board. Refer the document to [build and execute Zynq Emacps design](\ref page_zynq_emacps)
on openPOWERLINK.

# Dual processor design - Zynq Hybrid

One of the ARM core runs the openPOWERLINK MN user layer stack with demo application,
which is the processing system (PS). The time-critical kernel part of the stack
is running on a Microblaze softcore processor as a bare metal application, which
is running on the programming logic (PL) of SoC processor. The user and kernel layer exchanges
the data and control information using a shared memory interface.

This design requires an additional hardware i.e. AVNET expander board (AES-FMC-ISMNET-G)
which is connected to the FMC connector of Zynq ZC702 evaluation board. The AVNET
expander board has dual 1588 compatible 10/100 Ethernet PHYs to CAN, RS232 and RS48,
which enables to get start with Xilinx FPGA and SoC based designs. Refer the document to
[build and execute Zyny Hybrid design](\ref page_zynq_hybrid) on openPOWERLINK.