#!/bin/sh
# ----- Execute this script within the Nios II Command Shell   -----
# ----- It builds the FPGA bitstream for EBV DBC3C40           ------
sopc_builder --generate=1 niosII_openMac
quartus_cmd nios_openMac.qpf -c nios_openMac.qsf
