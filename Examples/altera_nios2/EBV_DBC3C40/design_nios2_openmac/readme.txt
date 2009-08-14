  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - FPGA design for EBV DBC3C40 with OpenMAC
	=========================================================


Contents
---------

- FPGA design with Nios II CPU and openMAC


Requirements
-------------

- Development Board EBV DBC3C40 (Mercury Board)

- Altera Quartus II v9.0 or newer (Web Edition is also possible)

- Altera Nios II Embedded Design Suite v9.0 or newer

- POWERLINK network as described in main readme.txt


Required files
---------------

- <project>.qpf (Quartus II project file)
- <project>.qsf (Quartus II settings file)
- <sopc_builder_system>.sopc (SOPC Builder system description)
- <sopc_builder_system>.sopcinfo (SOPC Builder report file)
- <toplevel>.bdf (top level schematic)
- <toplevel>.vhd (top level VHDL)
- EPL/* (VHDL and TCL source files of openMAC)
- <project>.sof (SRAM source file; generated, but needed for delivery)
- <megafunction_block>.qip, <megafunction_block>.bsf,
  <megafunction_block>.ppf, <megafunction_block>.vhd
- altpll0.qip, altpll0.bsf, altpll0.ppf, altpll0.vhd
- *.tcl


Generated files and subdirectories
-----------------------------------

- *.stp (SignalTap II file, needs special license)
- db/* (Compilation database files)
- incremental_db/* (Incremental database files)
- <sopc_builder_system>.qip
- altpll0.cmp
- cpu_0.sdc, cpu_0.vhd, cpu_0_*.mif, cpu_0_*.vhd
- EPL_openMAC_0.vhd, jtag_uart_0.vhd
- *_assignment_defaults.qdf ???
- .sopc_builder/*
- *.qif ???


How to build the binary
------------------------

1. Open "Nios II Command Shell"

2.


How to run the demo
--------------------

1. Setup the POWERLINK network as described in main readme.txt

2.

