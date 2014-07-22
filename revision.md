Revision history of openPOWERLINK Protocol Stack {#page_revision_history}
================================================

[TOC]

# Release 2 {#sect_revision_v2}

## V2.0.1 {#sect_revision_v2_0_1}

This is the final release of openPOWERLINK V2.0.1. No new features are
introduced within this version. It contains only fixes and optimizations
as well as some backported features from V1.8.4.

### Fixes:
- Fix error "invalid handle" which occurs when CFM initializes the network
- Fix setup of PReq filter in dllk_presChainingDisable()
- Fix compilation issue of edrv-i210.c on Linux 2.6.33
- Fix NMT MN timeout calculations
- Fix NMT command handling to avoid potential deadlocks with PRes chaining nodes
- Fix extended NMT command handling on CNs
- Fix sending of SyncRequests in NMT MN module
- Fix timer overflow in generic timer module
- Fix kernel heartbeat errors with small cycle times
- Fix "stack off" signalization in demos
- Fix feature flags to show support of ext. NMT commands
- Fix SDO sequence layer use count underflow
- Fix missing target-linuxkernel.c in Linux kernel module compilation
- Add missing prescaler suboject in CiA401_CN XDD
- Fix path issue in microblaze simpleboot script
- Increase default values for request queues to avoid buffer overflows on MN
  with more than 10 nodes
- Several minor fixes in header and source files
- Several fixes, cleanups and improvements in the documentation

### Changes:
- Optimize searching for OD entries
- Optimize eventk_process()

### Additions:
- Add seeting of thread names on Linux
- Add debug code to retrieve maximum used circbuffer size
- Add target_getTickCount() for Linux and Windows

### Removals:
- Remove outdated VxWorks documentation

## V2.1.0-pre1 {#sect_revision_v2_0_0_pre1}

This is an early preview of the upcoming openPOWERLINK release 2.1.0.

### Changes:

- Xilinx Zynq MN demo with Microblaze and openMAC in FPGA-part (PL)

### Known Issues:

- Xilinx Zynq MN:
    - When following the steps in [xapp1093](http://www.xilinx.com/support/documentation/application_notes/xapp1093-amp-bare-metal-microblaze.pdf)
      to run Microblaze without FSBL XMD-connect fails to stop/reset Microblaze.
      Usually this error can be ignored, debugging Microblaze is possible although.
      Otherwise, debugging the design with using the FSBL works.

    - The address map to HP0 AXI slave port of PS is not forwarded to system.xml
      by XPS automatically. Therefore, the `system.xml` in
      `hardware/boards/xilinx-z702/mn-dual-shmem-gpio/sdk` is used instead.
      If any changes are done in XPS (e.g. add some IP-Core, change addresses or
      add Chip Scope instance), the `system.xml` in `hardware/boards/xilinx-z702/mn-dual-shmem-gpio/sdk`
      must be revised accordingly.

## V2.0.0 {#sect_revision_v2_0_0}

This is the final release of openPOWERLINK V2.0.0.

### Changes:

- Selection of openCONFIGURATOR project for MN demos in CMake configuration
- Added error signaling initialization on MN
- Backporting of features from V1.08.3 and V1.08.4
  - MN support for extended NMT commands
  - Notification about PDO change events
  - Ethernet driver (edrv) for Intel I210 added
  - Implemented time triggered sending together with I210 driver
- Code clean-up
- Documentation clean-up
- Several bugfixes

### Known Issues:

- Altera FPGA MN:
  - After an up and running CN is reconnected to the network the FPGA MN is not able
    bringing the node to operational again.

    __Solution__:
      - Restart the host processor (e.g. in demo_mn_embedded `make download-elf`).
      - If the error resists, reprogram FPGA also (in drv_daemon `make download-all`).

  - In case of short cycle times the FPGA MN enters off state due to full queues.

    __Possible errors__: kErrorNoResource, kErrorEventPostError and kErrorDllAsyncTxBufferFull

    __Solution__:
    - Restart the host processor (e.g. in demo_mn_embedded `make download-elf`).
    - If the error resists, reprogram FPGA also (in drv_daemon `make download-all`).
    - Hold the MN in PreOp_1 until all nodes are configured or
    - Start up the network stepwise (connect first 2 CNs, then another
      2 CNs and so on...)

## V2.0.0-b1 {#sect_revision_v2_0_0_b1}

This is the first beta version of the openPOWERLINK release 2.0.0.

- The file system is slightly restructured and cleaned up
- Source Code is further refactored and cleaned up
- Header files are clearly separated into internal and external files
- The build system is separated for the stack, the hardware (fpga), the
  drivers and the demo applications
- Stack is compiled as static/dynamic libraries which will be linekd by all
  applications
- Support for CMake cross-compilation is added
- Xilinx build uses CMake now
- FPGA IP cores are updated
- Documentation is improved
- Stack configuration is simplified

## V2.0.0 pre1 {#sect_revision_v2_0_0_pre1}

This is an early preview of the upcoming openPOWERLINK release 2.0.0.
The new code base works fairly stable on Linux, Windows and FPGA platforms.
A lot of modules are refactored and cleaned-up but there are still several
files which need to be reformated and renamed. The release should give
a good overview on the new architecture and directory structure and can be
used for testing. It is not advised to use it in productive environments!

- [TASK] Update version information
- [TASK] Update Altera demos to Quartus version 13.0sp1
- [TASK] Update openCONFIGURATOR project Demo_3CN
- [FIX] Fix wrong doxygen reference in obd.h
- [TASK] Add libraries to the software manual
- [TASK] Change documentation options in doxyfile
- [TASK] Remove EplApiLinux.h
- [TASK] React on termination signals in demo_cn_console
- [TASK] React on termination signals in demo_mn_console
- [TASK] Add support to receive termination signals in the system module
- [TASK] Add Lcd to Altera MN demos
- [TASK] Revise FPGA CN demos by using process image functions
- [TASK] Add gitignore files to Xilinx demos
- [FIX] Change Nios II /f data cache line size to 4 byte
- [FIX] Fix FPGA MN and CN platform documentation
- [TASK] Add inline macros to hostif library for Nios II target
- [TASK] Add documentation for OBD module
- [TASK] Cleanup OBD macros
- [TASK] Rename EplObdMacro.h to obdmacro.h
- [TASK] Reformat EplObdMacro.h
- [TASK] Redesign OBD initialization
- [TASK] Refactor OBD module
- [TASK] Remove EPL_USE_DELETINST_FUNC
- [TASK] Refactor obd.h
- [TASK] Rename structure member variables in obd.h
- [TASK] Rename types and variables in obd.h
- [TASK] Reformat OBD functions
- [TASK] Rename OBD functions
- [TASK] Remove multi instance functionality from obd.c
- [TASK] Rename EplObd.h to obd.h
- [TASK] Merge OBD user and kernel module
- [TASK] Revise section mapping for Nios II MN
- [TASK] Add additional documentation
- [TASK] Improve documentation
- [TASK] Update revision information and add it to software manual
- [TASK] Add license file to software manual
- [TASK] Rework module documentation
- [FIX] Fix documentation of event and error handler modules
- [FIX] Fix documentation of DLL CAL source files
- [TASK] Add documentation of coding rules
- [FIX] MN/CN demo apps: Improve state change output
- [FIX] Add missing 'break' in demo_mn_qt
- [TASK] Remove dllcal and errhndcal shared buffer implementations
- [FIX] Fix issue with misssing images in doxygen documentation
- [FIX] Fix doxygen issues
- [TASK] Refactor processimage-cia302.c
- [TASK] Rework Lcd for Altera platform
- [TASK] Rename and move EplApiProcessImageSetup.c
- [TASK] Refactor API module
- [TASK] Move non API code to ctrl module
- [FEATURE] Add lock to Nios II processor targets
- [FEATURE] Add lock arch-module for Nios II
- [FIX] Fix typos in dllkframe.c
- [TASK] Rename circbuf-linuxshm.c to circbuf-posixshm.c
- [TASK] Use general MN console event source
- [FIX] Revise script for Nios II demos to preserve path in object files
- [TASK] Assign CPU ID to Nios II processors
- [TASK] Refactor OBD CDC module
- [TASK] Rename EplObdCdc.h
- [FIX] Change size of JTAG UART FIFO size
- [FIX] Disable C++ support in Nios II BSP
- [FEATURE] Add parameter setting to select FPGA Download cable
- [FEATURE] Add useful rules to generated Makefile for Nios II demos
- [FIX] Solve make download-elf issue with multi Nios II processors
- [TASK] Add arguments to create-this-app scripts to force settings
- [TASK] Relocate SYSTEC FPGA demo design to staging directory
- [TASK] Revise direct I/O demo software
- [TASK] Rework Altera FPGA MN and CN designs for TERASIC DE2-115
- [FEATURE] Add binary to seven segment LED decoder
- [FIX] Cleanup of Nios II Eclipse files
- [FIX] Solve permission denied issue with linker.x fix-script
- [FEATURE] Create common Qsys subsystems
- [FEATURE] Add possibility to skip the full Quartus compilation
- [FIX] Solve missing update of Ident-/Status-Response in kNmtCsReadyToOperate
- [FIX] Fix filter setup of unspecified request
- [FIX] Use logical OR for MAC comparison in edrv modules.
- [TASK] Refactor user LED module - Rename includes
- [TASK] Refactor user LED module
- [TASK] Rename CFM include files
- [TASK] Refactor CFM module
- [TASK] Refactor virtual ethernet driver
- [TASK] Refactor dll.h
- [FIX] Fix kernel module to compile on kernels > 3.7.0
- [TASK] Remove generated files in INK demo designs
- [TASK] Rename EplDll.h and update references
- [FIX] Fix doxygen issues for DLLK
- [TASK] Add DLLK module documentation
- [TASK] Rework DLLK - Cleanup header files
- [TASK] Rework DLLK - Refactor code
- [TASK] Rework DLLK - Rename external functions
- [FIX] Add return ability to create-this-fpga scripts
- [FIX] Add execution permission to create-this-app scripts
- [FIX] Revise section definitions according to previous changes
- [FIX] Fix Quartus project settings of directI/O example
- [TASK] Update FPGA MN Nios II designs
- [FIX] Change FPGA MN SW projects according to previous changes
- [FIX] Revise pdocal for host interface
- [FIX] Solve queue processing hang in host interface library
- [FIX] Add missing event interface for hostif
- [TASK] Revise Xilinx DirectI/O example software
- [TASK] Revise Altera DirectI/O example software
- [FEATURE] Add noos implementation for new event handler
- [TASK] Add placeholder ATOMIC_EXCHANGE for Nios II and Microblaze
- [FIX] Use target function for circular buffer (un-)lock
- [TASK] Add Xilinx Microblaze target
- [FIX] Omit obsolete eventcal.h
- [TASK] Remove DLL user module
- [TASK] Refactor EplNmt.h
- [TASK] Rework NMT kernel module
- [TASK] Remove NMT CAL modules
- [TASK] Rework NMT user module
- [TASK] Rework NMT CNU module
- [TASK] Rework user sync module
- [TASK] Remove SOPC design design_nios2_diretIO
- [TASK] Add QSYS Direct/IO FPGA design
- [FIX] Adapt Altera NiosII Direct/IO documentation
- [TASK] Rework NMT MNU module (merge)
- [TASK] Rework NMT MNU module (10)
- [TASK] Rework NMT MNU module (9)
- [TASK] Rework NMT MNU module (8)
- [TASK] Rework NMT MNU module (7)
- [TASK] Rework NMT MNU module (6)
- [TASK] Rework NMT MNU module (5)
- [TASK] Rework NMT MNU module (4)
- [TASK] Rework NMT MNU module (3)
- [TASK] Rework NMT MNU module (2)
- [TASK] Rework NMT MNU module (1)
- [FIX] Cleanup demo_mn_console
- [TASK] Rework statusu module
- [TASK] Rework identu module
- [TASK] Update event CAL documentation in modules.txt
- [FIX] Fix compilation problems on Windows
- [TASK] Rename Linux kernel load/unload scripts
- [FEATURE] Implement Linux kernel Driver
- [TASK] Adapt userspace daemon cmake files
- [TASK] Adapt powerlink libraries for new files
- [FEATURE] Rework PDO implementation
- [FEATURE] Rework event handler
- [TASK] Add new functions for kernel control module
- [TASK] Rework debug.c
- [FEATURE] Rework DLL CAL module
- [TASK] Update Linux X86 documentation
- [TASK] Remove PosixFileLinuxKernel.c
- [TASK] Add circular buffer definitions in EplDef.h
- [TASK] Add new include directories in CMakeLists.txt
- [TASK] Change Linux driver/device names
- [FEATURE] Add circular buffer library
- [FIX] Cleanup documentation in cmake.md and readme.md
- [TASK] Move unused header files of fec edrv into staging
- [FEATURE] Add additional control CAL module implementations
- [TASK] Cleanup control modules
- [FEATURE] Add new functions for error handler CAL
- [FEATURE] Add macros for atomic exchange operation
- [FIX] Remove semicolons at macro calls to avoid compiler warning
- [FIX] Add missing init/deinit of virtual ethernet in ctrlk.c
- [TASK] Add types for generic function pointers in global.h
- [FIX] Add missing includes in trace-printk.c
- [TASK] Add new definitions in EplInc.h
- [FIX] Fix CMakeLists.txt for demo_mn_console
- [FIX] Correct typos in demo_mn_console/main.c
- [FIX] Add missing variables for CN with cross-traffic
- [FIX] Cleanup Altera MN documentation
- [FIX] Add missing include file to event.c
- [TASK] Revise section defines Nios II as Managing Node
- [TASK] Add CMACRO to differ between MN and CN
- [FEATURE] Add scripts to FPGA MN designs
- [TASK] Rebuild MN board examples due to previous fix
- [FIX] Dynamic buffer address write works with 16 bit host interface
- [TASK] Split Altera Nios II documentation into MN and CN
- [FIX] Add missing include file to pdokcal-hostif.c
- [TASK] Cleanup demo_mn_console
- [FIX] Remove unneeded edrvcyclic.c from userspace library compilation
- [FEATURE] Add doxygen configuration files for demos
- [FIX] Replace CDC to TXT translator with PERL script
- [TASK] Add parallel host interface demo designs to SW projects
- [TASK] Update dual Nios MN demo design
- [FEATURE] Add demo design for asynchronous multiplexed parallel host interface
- [TASK] Update Host Interface IP-Core with asynchronous parallel interface
- [TASK] Revise Nios II MN daemon to trace library rework
- [TASK] Revise Nios II MN Demo Application to trace library rework
- [TASK] Adapt Host Interface control module to new target sleep function
- [FIX] Avoid compiler warning with target-nios2.c
- [FIX] Apply fixes for removing compiler warnings
- [FIX] Fix compilation of Linux powerlink userspace daemon
- [TASK] Clean up QT demo and adapt to new stack architecture
- [TASK] Clean up console demos for new stack architecture
- [FEATURE] Add openPOWERLINK Linux userspace daemon
- [TASK] Adapt openPOWERLINK library to new CMake files
- [FEATURE] Add make files for openPOWERLINK user library
- [TASK] Adapt and cleanup general CMake files
- [TASK] Extract common functions of demos
- [FEATURE] Add getopt command line parser library
- [TASK] Adapt control module to new target sleep function
- [TASK] Extract trace functions into separate library
- [FEATURE] Extract console functions into separate library
- [TASK] Remove trace output in EplApiProcessImageSetup.c
- [TASK] Remove warning option for microsoft compiler in obdcdc.c
- [TASK] Add function api_waitSyncEvent()
- [TASK] Reworked target specific stack functions
- [FIX] Fix timeouts in pdoucalsync-bsdsem.c
- [FIX] Grey out unused code in Altera Nios II Eclipse
- [FIX] Fix heartbeat function for direct control module
- [FEATURE] add FPGA MN 10 CN openCONFIGURATOR project
- [FEATURE] Add Altera Nios II MN Host Demo
- [FIX] Control interrupt from Host Interface IP-Core through ctrlucal
- [FEATURE] Add Nios II Pcp SW project for POWERLINK MN daemon
- [FEATURE] Add Host Interface IP-Core CAL interface
- [FEATURE] Add Host Interface IP-Core SW library
- [FEATURE] Add Terasic DE2-115 MN dual Nios II board example
- [FEATURE] Add Host Interface IP-Core V0.0.1 for Altera FPGA
- [TASK] Adapt doxygen file to new directory structure
- [TASK] Add Nios II console IO primitives
- [TASK] Add Nios II target implementation for platforms without Shared Buffer
- [FIX] Avoid compiler error with Altera Nios using obdcdc.c
- [TASK] Adapt VxWorks demo to new directory structure
- [FIX] Fix AppcbEvent in demo_cn_console to compile on windows
- [TASK] Adapt general examples and libs to new directory structure
- [FIX] Adapt Altera and Xilinx examples to new directory structure
- [TASK] Revise omethlib and openMAC Ethernet driver
- [TASK] Adapt sources to new directory structure and naming conventions
- [FIX] Clean direct event implementation for dual processor solution
- [FIX] Add missing source file in CMakeList.txt for powerlink lib
- [FIX] Check kernel layer heartbeat and status
- [TASK] update Altera and Xilinx directio example to control module
- [FIX] Revise system.h macros for POWERLINK IP-Core if added to Qsys
- [TASK] Provide process function also in ctrlu and ctrlucal modules
- [TASK] Provide process function also in ctrlkcal module
- [FIX] Add ifdefs to control module for not using SharedBuff
- [FIX] Omit unneeded SharedBuff include in common header file
- [TASK] Adapt X86 demos and libraries to new directory structure
- [TASK] Adapt VxWorks demo to new control and PDO module
- [FIX] Remove unneeded includes from ctrlkcal-direct.c/ctrlucal-direct.c
- [TASK] Add API function for checking kernel stack
- [FEATURE] Add control module
- [TASK] Add direct calling of sync callback in pdoXcal_syncnull
- [FIX] Add missing module include macro in EplInc.h
- [FIX] Fix calling convention for user error handler functions
- [FIX] Add PUBLIC calling convention for process image api functions
- [TASK] Change calling convention of EplTimeruProcessThread
- [FIX] Fix CMakeList.txt for generic powerlink lib
- [TASK] Move unported demos into 'unported' directory
- [FEATURE] enable pdo sync depending on nmt state
- [FEATURE] forward sync callback from application to pdo user part
- [TASK] Change doxygen options
- [TASK] update Xilinx directIO examples
- [TASK] update Altera directIO examples
- [TASK] update POWERLINK IP-Core to V0.2.9
- [TASK] delete EBV DBC3C40 example
- [FIX] Add missing shutdown of errhnducal in errhndu
- [FIX] Fix cleaning of PDO memory in pdou.c
- [FIX] Add correct detection of exisiting shared buffer
- [TASK] introduced download-all command for Xilinx demo
- [TASK] adjust revised directory structure in Altera and Xilinx demos
- [TASK] revised fpga directory for a clear overview
- [FIX] Remove unused EplObdkCal.c/h
- [TASK] update Altera and Xilinx directio example to refactored PDO module
- [FEATURE] add PDO local implementation for single process platforms
- [FIX] Fix error handler
- [TASK] Move init/shutdown of pdoXcal in pdoX
- [TASK] Rework ShbIpc-LinuxUser
- [FIX] Fix behaviour of ShbCirSetSignalHandlerNewData()
- [TASK] Cleanup event handler
- [FIX] Fix unit test compilation
- [TASK] Change doxygen configuration
- [TASK] Cleanup error handler
- [TASK] Add new macros for conditional module integration
- [TASK] Adapt X86 powerlink library and demo applications for new PDO
- [TASK] Refactor PDO modules - refactoring
- [TASK] Refactor PDO modules - renaming files
- [TASK] Improve documentation
- [FIX] Fix setting of macro EPL_USE_SHAREDBUFF
- [FIX] Fix wrong references to dllkcal
- [TASK] Rework documentation
- [TASK] Refactor error handler
- [TASK] Add calling of PostDefault callback at parameter load
- [TASK] Add unit tests for event module
- [TASK] Refactor event handler - Refactor and cleanup
- [TASK] Refactor event handler - Rename Files
- [FEATURE] revised event handling approach with cal
- [TASK] Refactor DLL CAL module (3)
- [TASK] Refactor DLL CAL module (2)
- [TASK] Refactor DLL CAL module (1)
- [TASK] adjust new directory structure in Altera and Xilinx demos
- [TASK] create fpga and Examples/arch for fpga demos
- [FEATURE] add benchmark IO for qsys subsystem "pcp_0"
- [FIX] MN allocates asynchronous slot to itself but sends no message
- [TASK] update Xilinx directIO examples
- [TASK] update Altera directIO examples
- [FEATURE] Move functions from interrupt context to internal memory
- [TASK] update openMAC drivers to V0.2.7
- [TASK] update Xilinx POWERLINK IP-Core to V0.2.7
- [TASK] update Altera POWERLINK IP-Core to V0.2.7
- [FIX] Replace "winpcap" by "pcap library" in the CN console demo
- [TASK] Implement macro UNUSED_PARAMETER()
- [TASK] Add bit-sized integer type macros
- [TASK] Add trace macros for PDO module
- [FIX] Add missing definitions for xilinx platform in global.h

# Release 1 {#sect_revision_v1}

## V1.8.2 {#sect_revision_v1_8_2}

- [TASK] Set PRes RD Flag to 1 even if payload is 0
- [FIX] Invalid state change of CN after "NMT_StopNode" command.
- Linux EplLoad script: fix error message
- [Feature] Transmit SoC relative time by MN.
- [FIX] Fix Linux x86 documentation
- [FIX] X86 QT demo application: Ethernet device name has to be stored
  in global variable.
- [FIX] Fix typos.
- [TASK] Apply Allman-style.
- [TASK] Improve simple formating issues.
- [FEATURE] Add absolute time field to TX buffer for openMAC
  (delete duplicate edrv.h from openMAC driver directory).
- [TASK] Create documentation for Altera and Xilinx, change global readme.txt
- [FIX] Add Xilinx support into Benchmark.h
- [FIX] Enhance xilinx usleep to be more accurate
- [FIX] Update EdrvOpenmac
- [FEATURE] Update Altera POWERLINK IP-Core to version 0.2.5
  and move it to common directory for all demo project.
  Refer to this directory via .ipx files.
  Update and rebuild corresponding demo projects.
- [FEATURE] Update Xilinx POWERLINK IP-Core and move it to common directory
  for all demo project. Update corresponding demo projects.
- [TASK] Improve EplDebug to provide a function for values of type tEplKernel.
- [TASK] Use new EplDebug functions in CN and MN demo applications.
- [TASK] Add a function to the EplDebug module for emergency error code
- [FIX] Add PUBLIC macro to EplDebug functions
- [TASK] Use EplDebug function for emergency error codes in MN demos
- [FIX] Incorrect multiplexed (MS) flag in PRes
- [FIX] Dllk: Avoid zero sized array for CNs
  (Fix compile errors when EPL_NMT_MAX_NODE_ID is 0)
- [FEATURE] Add X86 CN demo application
- [TASK] Add support for Linux 3.x kernel
- [FIX] fix CMakeLists.txt for QT MN app
- [FIX] Prevent compile warning in EplObd.c
- [FIX] Update CN OD default values to fit to openCONFORMANCE test
- [FIX] Obd/SdoSequ: Implement object 0x1300 (SDO Sequence layer timeout)
- [FIX] Replace "printf" with appropriate makros to prevent SDO delay issues
- [TASK] Remove reference to unused target system in EplObdCdc.c
- [TASK] Remove comment which shows Visual C warning in EplObdCdc.c
- [TASK] Remove unused architectures from global.h
- [TASK] Cleanup global.h
- [FIX] Fix Xilinx DEV_SYSTEM definitions
- [TASK] Add additional files to .gitignore
- [TASK] Remove Linux demo_qt application
- [TASK] Rename generic demo_process_image_console to demo_mn_console
- [TASK] Rename demo_process_image_qt to demo_mn_qt
- [TASK] Rename VxWorks demo_process_image_console to demo_mn_console
- [TASK] Simplify configuration options in CMake
- [TASK] Add some cleanups in demo_mn_console and demo_kernel
- [FEATURE] Add generic Asnd API
- [TASK] use variadic macros for PRINTF/TRACE macros
- [FIX] Add mapping objects for 40 CNs to the CiA302-4_MN OD and XDD
- [FIX] Remove PRes frame size check for node running as CN

## V1.8.1 {#sect_revision_v1_8_1}

- [FIX] Fix initialization of Edrv8139
- [TASK] Cleanup generic demos
- [TASK] Cleanup EplErrStr.c
- [FIX] Linux demo_kernel does not compile on x86_64
- [FIX] Unreadable network interfaces in QT demo on Windows
- [FIX] Correct syntax error in EplCfg.h
- [FIX] ShbLinuxKernel.h included in ShbIpc-NoOS.c
- [FIX] Linux pcap does not reconnect after lost link
- [FIX] QT demo misses CN reconnect
- [FIX] SPIN_LOCK_UNLOCKED is undefined on recent Linux kernels
- [FIX] QT demo network interface string is not passed with correct type
- [FEATURE] Add additional devices to Edrv82573: 82573L, 82567V, 82583V, 82567LM
- [FEATURE] Update of the Xilinx examples for Avnet LX150T (IEK)
            and LX9 MicroBoard with new POWERLINK IP-core
- [FEATURE] Update of the Altera example for EBV DBC3C40 board
            with the new POWERLINK IP-core
- [FEATURE] Added the TERASIC INK board demo to the Altera FPGA demos
- [FEATURE] Add Intel 8255x 100MBit Edrv for Linux
- [FIX] Remove cross-traffic mapping in the FPGA CN demos
- [FIX] Adjust the MN and CN XDD files
- [FIX] Windows projects can use EplSdoUdpu.c,
  this also fixes demo_pi_console compile errors in VS2010
- [FEATURE] Add support for Intel 82574L ethernet device
- altera_nios2: Use Altera Enhanced Interrupt API
- [FEATURE] Update of the Altera example for SYS TEC ECUcore-EP3C board
            with the new POWERLINK IP-core

## V1.8 {#sect_revision_v1_8}

- Enable PResChaining in Altera FPGA demo project
- Fix ready flag of TPDO on MN and CN
- VirtualEthernet: add capability of setting the MTU
- Forward new event kEplNmtBootEventEnableReadyToOp on CN to application
- MN: Update object 0x1F8E also in case of no IdentResponse
- MN: fix uninitialized variable bNmtState in function EplNmtMnuCheckNmtState
- Rename cvsignore to gitignore
- Add support for VxWorks on X86
- SDO Command Layer: Fix SDO segmented transfer data size calculation
- Modifications to compile the stack as Windows DLL
- Adjust header files to correctly resolve dependencies with VC10
- Use CMake for Windows / Visual Studio target (library and console demo)
- Update of WdpPack/WinPcap to 4.1.2
- Joined Windows and Linux version of QT demo, powerlink_user_lib and demo_process_image_console
- Fix several typos in source files
- MN: Reset flags if node is in CsStopped.
- Add EplErrStr module provided by E. Dumas
- SDO Sequence Layer: Fix handling of unsupported SDO sequence layer protocols.
- Obd: Remove unused float macros from stack sources.
- Obd: Set default value for object 0x1F8C to 0x1C (NotActive).
- NmtMnu: Update object 0x1F8E/240 when MN changes state.
- Add Xilinx/Microblaze platform to the stack (projects for Avnet LX150T (IEK) and LX9 MicroBoard)
- Changes according to EPSG DS 301 V1.1.0
  * Correct definition of 0x1C00 and 0x1C02
  * Correct name and default value of object 0x1C14
  * Correct default value of 0x1C0F
  * Correct default values of 0x1F89/03
  * Correct default value of 0x1F8C
- EdrvFec: Add Ethernet driver for Freescale FEC (i.MX and some Coldfire)


## Older Releases {#sect_revision_older}

The revision log of older versions is removed. Please refer to the git history
or an older stack version to get the revision history

