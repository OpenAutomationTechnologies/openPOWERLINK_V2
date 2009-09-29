  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - FPGA design for EBV DBC3C40 with openMAC
	=========================================================


Contents
---------

- FPGA design with Nios II CPU and openMAC


Requirements
-------------

- Development Board EBV DBC3C40 (Mercury Board)

- Altera Quartus II v9.0 or newer (Web Edition is also possible)

- Altera Nios II Embedded Design Suite v9.0 or newer

- Experiences with this development environment are required


How to build the design (generate the SOF file)
------------------------------------------------

These steps are only necessary if you want to change the FPGA design.
Otherwise you can use the supplied SOF file and go directly to step 6.

1. Open the Quartus project file nios_openMac.qpf with Altera Quartus II.

2. Open the SOPC Builder via menu "Tools" -> "SOPC Builder".

3. Press the button "Generate" in the SOPC Builder to regenerate the Nios II system.

4. Close the SOPC Builder when the generation has finished.

5. Start the compilation in the Quartus II window via menu "Processing" -> "Start Compilation".

6. Use the design with the supplied demo projects in the openPOWERLINK
   subdirectory Examples\altera_nios2\no_os\gnu.

   Please refer to the readme.txt in the subdirectory of the demo project for
   further information.


Appendix
=========

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

