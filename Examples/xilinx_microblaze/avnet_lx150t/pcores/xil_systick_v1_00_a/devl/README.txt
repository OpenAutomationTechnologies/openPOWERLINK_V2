TABLE OF CONTENTS
  1) Peripheral Summary
  2) Description of Generated Files
  3) Description of Used IPIC Signals
  4) Description of Top Level Generics


================================================================================
*                             1) Peripheral Summary                            *
================================================================================
Peripheral Summary:

  XPS project / EDK repository               : C:\vhdl\xilinx\spartan6\openSlave_EPL02
  logical library name                       : xil_systick_v1_00_a
  top name                                   : xil_systick
  version                                    : 1.00.a
  type                                       : PLB (v4.6) slave
  features                                   : slave attachment
                                               user memory spaces

Address Block for User Logic and IPIF Predefined Services

  User logic memory space 0                  : C_MEM0_BASEADDR
                                             : C_MEM0_HIGHADDR


================================================================================
*                          2) Description of Generated Files                   *
================================================================================
- HDL source file(s)

  hdl/vhdl/xil_systick.vhd

    This is the template file for your peripheral's top design entity. It
    configures and instantiates the corresponding design units in the way you
    indicated in the wizard GUI and hooks it up to the stub user logic where
    the actual functionalites should get implemented. You are not expected to
    modify this template file except certain marked places for adding user
    specific generics and ports.

  vhdl/user_logic.vhd

    This is the template file for the stub user logic design entity, either in
    VHDL or Verilog, where the actual functionalities should get implemented.
    Some sample code snippet may be provided for demonstration purpose.

- XPS interface file(s)

  data/xil_systick_v2_1_0.mpd

    This Microprocessor Peripheral Description file contains information of the
    interface of your peripheral, so that other EDK tools can recognize your
    peripheral.

  data/xil_systick_v2_1_0.pao

    This Peripheral Analysis Order file defines the analysis order of all the HDL
    source files that are used to compile your peripheral.

- Other misc file(s)

  devl/ipwiz.opt

    This is the option setting file for the wizard batch mode, which should
    generate the same result as the wizard GUI mode.

  devl/README.txt

    This README file for your peripheral.

  devl/ipwiz.log

    This is the log file by operating on this wizard.


================================================================================
*                         3) Description of Used IPIC Signals                  *
================================================================================
For more information (usage, timing diagrams, etc.) regarding the IPIC signals
used in the templates, please refer to the following specifications (under
%XILINX_EDK%\doc for windows or $XILINX_EDK/doc for solaris and linux):
proc_ip_ref_guide.pdf - Processor IP Reference Guide (chapter 4 IPIF)
user_core_templates_ref_guide.pdf - User Core Templates Reference Guide

Bus2IP_Clk
    Synchronization clock provided to the user logic. All IPIC signals are 
    synchronous to this clock. It is identical to the input <bus>_Clk signal of 
    the peripheral. No additional buffering is provided on the clock; it is 
    passed through as is. 

Bus2IP_Reset
    Active high reset for used by the user logic; it is asserted whenever the 
    <bus>_Rst signal asserts or whenever there is a software-programmed reset 
    (if the soft reset block is included). 

Bus2IP_Addr
    Address bus to the user logic indicating the desired address of the 
    requested read or write operation. It can be used for additional address 
    decoding or as input to addressable memory devices. 

Bus2IP_CS
    Active high chip select bus. Assertion of a chip select indicates an active 
    transaction request to the chip select's target address space. Typically 
    used for user logic memory space selection. 

Bus2IP_RNW
    This is an input signal to the user logic; it indicates the sense of a 
    requested operation with the user logic. High is a read and low is a write. 
    It is valid whenever at least one of the Bus2IP_CS bits is active. 

Bus2IP_Data
    Write data bus to the user logic. Write data is accepted by the user logic 
    during a write operation by assertion of the write acknowledgement signal 
    and the rising edge of the Bus2IP_Clk. 

Bus2IP_BE
    Byte Enable qualifiers for the requested read or write operation to the user 
    logic. A bit in the Bus2IP_BE set to '1' indicates that the associated byte 
    lane contains valid data. For example, if Bus2IP_BE = 0011, this indicates 
    that byte lanes 2 and 3 contains valid data. 

IP2Bus_Data
    Output read data bus from the user logic; data is qualified with the 
    assertion of IP2Bus_RdAck signal and the rising edge of the Bus2IP_Clk. 

IP2Bus_RdAck
    Active high read data qualifier providing the read acknowledgement from the 
    user logic. Read data on the IP2Bus_Data bus is deemed valid at the rising 
    edge of the Bus2IP_Clk and IP2Bus_RdAck asserted high by the user logic. For 
    immediate acknowledgement (such as for a register read), this signal can be 
    tied to '1'. Wait states can be inserted in the transaction by delaying the 
    assertion of the acknowledgement. 

IP2Bus_WrAck
    Active high write data qualifier providing the write acknowledgement from 
    the user logic. Write data on the Bus2IP_Data bus is deemed accepted by the 
    user logic at the rising edge of the Bus2IP_Clk and IP2Bus_WrAck asserted 
    high by the user logic. For immediate acknowledgement (such as for a 
    register write), this signal can be tied to '1'. Wait states can be inserted 
    in the transaction by delaying the assertion of the acknowledgement. 

IP2Bus_Error
    Active high signal indicating the user logic has encountered an error with 
    the requested operation. It is asserted in conjunction with the read/write 
    acknowledgement signal(s). 

================================================================================
*                     4) Description of Top Level Generics                     *
================================================================================
C_BASEADDR/C_HIGHADDR
    These two generics are used to define the memory mapped address space for
    the peripheral registers, including Soft Reset register, Interrupt Source
    Controller registers, Read/Write FIFO control/data registers, user logic
    software accessible registers and etc., but excluding those user logic
    memory spaces if ever existed. When instantiation, the address space
    size determined by these two generics must be a power of 2 (e.g. 2^k =
    C_HIGHADDR - C_BASEADDR + 1), a factor of C_BASEADDR and larger than the
    minimum size as indicated in the template.

C_SPLB_AWIDTH
    This is the slave interface address bus width for Processor Local Bus
    version 4.6 (PLBv46). Value can be assigned automatically by EDK
    tooling during system creation.

C_SPLB_DWIDTH
    This is the slave interface data bus width for Processor Local Bus
    version 4.6 (PLBv46). Value can be assigned automatically by EDK
    tooling during system creation.

C_SPLB_NUM_MASTERS
    This indicates to the slave interface the number of PLBv46 masters
    present. Value can be assigned automatically by EDK tooling during
    system creation.

C_SPLB_MID_WIDTH
    This indicates to the slave interface the number of bits required
    for the PLB_masterID input bus. It is an integer value equal to
    log2(C_SPLB_NUM_MASTERS). Value will be assigned automatically by
    EDK tooling during system creation.

C_SPLB_NATIVE_DWIDTH
    This indicates to the slave interface the native bit width of the
    internal data bus of the peripheral. Some peripheral will require
    the value of this parameter to be fixed, while others might have
    selectable native data widths.

C_SPLB_P2P
    This indicates to the slave interface when it is exclusively attached
    to a PLBv46 bus via a Point to Point interconnect scheme. In this
    scenario, the slave interface may be able to reduce resource utilization
    by eliminating address decode function and modifying interface behavior
    to allow for a reduction in latency.

C_SPLB_SUPPORT_BURSTS
    This indicates to the associated PLBv46 bus that this slave interface
    support burst transfers to improve performance.

C_SPLB_SMALLEST_MASTER
    This indicates the smallest native data width of any master on the
    corresponding PLBv46 bus that may access the slave interface. It allows
    optimizations within the slave interface logic if narrower masters don't
    have to be supported for that application.

C_SPLB_CLK_PERIOD_PS
    This is the period of the PLBv46 bus clock (in picoseconds) for the
    corresponding PLBv46 slave interface attachment. It has been defined
    for use by peripheral that needs to know the bus clock rate to improve
    certain functions such as internal timers.

C_INCLUDE_DPHASE_TIMER
    This indicates if the data phase timer is used or not. The value of
    0 will exclude the timer.  The value of 1 includes the timer.
    If C_INCLUDE_DPHASE_TIMER = 1 and after 128 SPLB_Clk cycles, as
    measured from the assertion of Sl_AddrAck, the User IP does not
    respond with either an IP2Bus_RdAck or IP2Bus_WrAck the 
    plbv46_slave_single will de-assert the User IP cycle request
    signals, Bus2IP_CS and Bus2IP_RdCE or Bus2IP_WrCE, and will assert
    Sl_rdDAck with Sl_rdDBus=zero for a read cycle or Sl_wrDAck for
    a write cycle. This will gracefully terminate the cycle. Note 
    that the requesting master will have no knowledge that the data
    phase of the PLB request was terminated in this manner.

C_FAMILY
    This is to set the target FPGA architecture, s.t. virtex5, etc.

C_MEMn_BASEADDR/C_MEMn_HIGHADDR (n = 0, 1, 2, etc.)
    These two generics are used to define the memory mapped address space for
    user logic memory space n, which are typically used in peripherals like
    memory controllers, bridges, that need to access memory blocks other
    than local register space. When instantiation, the address space size
    determined by these two generics should be a power of 2 (e.g. 2^k =
    C_MEMn_HIGHADDR - C_MEMn_BASEADDR + 1) and a factor of C_MEMn_BASEADDR.

================================================================================
*          5) Location to documentation of dependent libraries                 *
*                                                                              *
*   In general, the documentation is located under:                            *
*   $XILINX_EDK/hw/XilinxProcessorIPLib/pcores/$libName/doc                    *
*                                                                              *
================================================================================
proc_common_v3_00_a
	No documentation for this library

plbv46_slave_single_v1_01_a
	C:\vhdl\xilinx\spartan6\openSlave_EPL02\C:\Xilinx\11.1\EDK\hw\XilinxProcessorIPLib\pcores\plbv46_slave_single_v1_01_a\doc\plbv46_slave_single.pdf

