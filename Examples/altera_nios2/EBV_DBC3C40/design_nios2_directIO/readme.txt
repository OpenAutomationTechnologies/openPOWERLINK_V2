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
  and Altera Nios II Embedded Design Suite v9.0 or newer
  (http://www.altera.com/support/software/download/nios2/dnl-nios2.jsp )

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
