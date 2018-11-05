Revision history of openPOWERLINK Protocol Stack {#page_revision_history}
================================================

[TOC]

# Release 2 {#sect_revision_v2}

## V2.7.1 {#sect_revision_v2_7_1}

This is the latest release of the V2.7 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.7.1. For a detailed revision history
refer to the Git source code history.

### Changes:
- Add CN Zynq hybrid design
- Refactor omethlib
- Update openCONFIGURATOR project in stack
- Unify default ami module implementation
- Handle oplk_setNonPlkForward() API in OS designs
- Remove unused defines and according dead code

### Fixes:
- Fix Zynq Hybrid performance issue
- Use Peterson's algorithm for dualproshm lock
- Fix Linux PCIe compilation issue
- Fix Windows NDIS and PCIe basic run issue
- Doxygen Enhancement
- Fix output path for NDIS driver build
- Fix timesync issue in Zynq hybrid CN design
- Fix for RMN support in Zynq Emacps
- Fix compilation errors for Linux kernel > v4.15
- Fix for ifconfig deprecated issue
- edrv-rawsock_linux: add missing include for u_char type
- Fix errors in Altera Nios II build scripts
- Fix errors in the xdd/xdc files

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.7.0 {#sect_revision_v2_7_0}

This is the first release of the V2.7 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.6 series (\ref sect_revision_v2_6_1)

Following is a summary of changes in V2.7.0. For a detailed revision history
refer to the Git source code history.

### New Features:
- Add SoC timestamp forwarding feature in Windows NDIS design
- Add SoC timestamp forwarding feature in Windows PCIe design
- Add MN net time distribution feature implementation
- Add SoC time stamp forwarding feature in Linux PCIe design
- Add raw socket implementation
- Add Intel 82540EM support
- Enable SoC time forward in host interface stack libraries
- Add timesync support to timesync CAL modules
- Add timesync support to host interface driver
- Add timesync shared memory to host interface IP-Core
- Add SoC timestamp forwarding for Linux ioctl design
- Add SoC time stamp forwarding feature in BSD semaphore design
- Add SoC time stamp forwarding feature in Zynq Hybrid design

### Fixes:
- Fix Compilation issue in Windows NDIS 32 bit
- Fix variable naming in edrv-8255x.c
- Avoid comparison with TRUE
- Use pci_enable_msix_exact in edrv-i210
- Fix issues in veth-linuxkernel with kernel >= 4.11.9
- Add uaccess.h header to edrv
- Update format specifier for size_t
- Fix several Clang warnings
- Fix type conversion warning in windows design
- Fix typecasts and typos in the firmware manager module
- Fix commit message checking
- Fix CMake commands in documentation
- Fix Xilinx and Altera documentation
- Unblock connection attempts after ignored SDO commands
- Fix errors/warnings in Visual Studio
- Fix shutdown stack if still running during exit
- Fix startup/shutdown issues of the Windows kernel driver
- Fix driver compilation warning in NDIS intermediate design
- Add NDEBUG macro in Visual studio project for Windows NDIS
- Fix Windows 10 compatibility issues for Windows NDIS
- Fix SDO over UDP feature in Windows designs
- Fix data type definition in Windows designs for 64 bit systems
- Improve DLL_CNLossSoC_REC cumulative counter
- Use size_t for sizes in order to fix several 64 bit warnings
- Add conversion from us to ns for hrestimer
- Add 8111 edrv (rev 0c) support for kernel module design
- Fix doxygen warnings in timesyncucal modules
- Fix application compilation issue in 32 bit Windows
- More robust mutex init against previous crashes
- Fix wrong calculation of the error counters
- Fix OD and XDD for compliance with EPSG DS 301 V1.3.0
- Hostif IP-Core: Drop useless inclusion
- Use getopt() only on Windows
- Use pcap-config to fix static linking

### Changes:
- Use ami_setUint8Le() to honour the API
- Add 82567LM driver support for kernel module design
- Update net device name as plk_veth for all platforms
- Use generic data types in several stack modules
- Enable SoC timestamp forward feature for Windows PCIe and NDIS
- Improve code quality in ctrlucal modules
- Silently ignore invalid SDO commands for closed connections
- Add vera++ to Travis-CI
- Add Travis continuous integration
- Enable SoC timestamp forward feature for supported platforms
- Add API implementation of net time in windows target
- Add an API to get the current system timestamp in target files
- Increase tightly coupled memory size in Terasic DE2i-150
- Use network interface enumeration in demo applications
- Add network interface enumeration
- Refactor network interface card parameter passing
- Rename common files of Linux PCIe and Zynq hybrid designs

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.6.2 {#sect_revision_v2_6_2}

This is the latest release of the V2.6 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.6.2. For a detailed revision history
refer to the Git source code history.

### Changes:
- Silently ignore invalid SDO commands for closed connections
- Add 8111 edrv (rev 0c) support for kernel module design
- Update the company name of B&R

### Fixes:
- Fix OD and XDD for compliance with EPSG DS 301 V1.3.0
- Improve DLL_CNLossSoC_REC cumulative counter
- Fix wrong calculation of the error counters
- Fix conversion from us to ns in hrestimer
- Fix SDO over UDP feature in Windows designs
- Fix several compiler warnings

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.6.1 {#sect_revision_v2_6_1}

This is the latest release of the V2.6 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.6.1. For a detailed revision history
refer to the Git source code history.

### Changes:
- Add documentation for Linux Zynq Hybrid design
- Add documentation for Linux Zynq Emacps design
- Enable activation of modular device feature flag

### Fixes:
- Fix Windows NDIS driver compilation
- Fix fallback issue of CN from PreOp2 to PreOp1
- Refactor obdcdc and obdconf to avoid conversion warnings
- Fix warnings and issues with 64 bit systems
- Fix issues in firmware manager

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.6.0 {#sect_revision_v2_6_0}

This is the first release of the V2.6 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.5 series (\ref sect_revision_v2_5_2)

Following is a summary of changes in V2.6.0. For a detailed revision history
refer to the Git source code history.

### New Features:
- Add store/restore support to Nios II target
- Add Multiple Read / Write By Index for SDO server and client
- Add firmware manager to MN console demo

### Fixes:
- Correct PDO frame count statement in Altera CN documentation
- Fix loss of frames on SDO client segmented WriteByIndex
- Fix missing index of event kObdEvWrStringDomain
- Shutdown pcap thread using pthread_cancel()
- Reduce unconstraint pins FPGA warnings
- Delete store/restore XDD default values
- Fix alignment for reading store/restore capabilities
- Handle missing PRC support of a node
- Avoid flood of error traces in pdou_cbNmtStateChange()

### Changes:
- Update OD and XDD for compliance with EPSG DS 301 V1.3.0
- Update Linux platform documentation
- Refactor Qt demo
- Update openCONFIGURATOR sample projects
- Add CFI flash interface to cn-single-hostif-gpio FPGA demo
- Refactor SDO client and server sources
- Add NMT commands and events for firmware update support

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.5.2 {#sect_revision_v2_5_2}

This is the latest release of the V2.5 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.5.2. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Fix access to aTimerInfo in hrestimer for Linux Kernel
- Fix wrong size information provided by SDO callback
- Use error handler posix memory sources in driver daemon
- Fix contributor url
- Avoid integer overflow in searchIndex
- Resolve compilation issue in Zynq hybrid design

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.5.1 {#sect_revision_v2_5_1}

This is the latest release of the V2.5 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.5.1. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Update documentation for Xilinx Zynq designs
- Solve data freshness issue on Zynq Hybrid design
- Fix wrong assertion in eventu_postError
- Improve Nios II CN documentation
- Fix SKIP_BITSTREAM enable in Zynq Hybrid design
- Fix Microblaze BSP build in Zynq Hybrid design
- Document update for Altera C5 SoC design
- Fix build documentation for Vivado under Windows
- Resolve xilinx-z702 PCP compiler warnings in Zynq hybrid
- Resolve kernel panic for RMN
- Remove unimportant targetsection assignments
- Fix typo in known issues documentation
- Reduce timeout steps in ctrlucal_executeCmd for dualprocshm CAL
- Add support for VEth in Zynq hybrid design
- Fix interface driver probe functions for Zynq hybrid design
- Fix warnings in CMake version 3.3.x
- Fix SDO server for non-numeric expedited WriteByIndex
- Fix missing kObdEvPostWrite event for non-numeric objects
- Fix NULL pointer used in edrv-i210 probe
- Increase Tx buffer to handle higher PDO size in Zynq emacps
- Fix driver probe functions for Zynq emacps
- Update cmake configuration in edrv module
- Fix NULL pointer used in edrvcyclic and sdotest-com
- Remove unreachable break
- Correct local variables
- Add missing "_p" for parameters in obdconf
- Add missing includes
- Remove \return for functions that return nothing
- Veth: Fix compilation on Linux kernels >= 4.7

### Changes:
- Consistency cleanup in build documentation
- Update required Doxygen
- Update Xilinx Zynq documentation
- Improve readability of the directories documentation
- Clean up kernel interface driver for Zynq hybrid design

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.5.0 {#sect_revision_v2_5_0}

This is the first release of the V2.5 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.4 series (\ref sect_revision_v2_4_1)

Following is a summary of changes in V2.5.0. For a detailed revision history
refer to the Git source code history.

### New Features:
- Port Zynq PCAP design to Vivado 2016.2
- Port Zynq emacps design to Vivado 2016.2
- Port Zynq + Microblaze (Hybrid) design to Vivado 2016.2
- Add Linux Kernel driver for Zynq Hybrid design
- Add MinGW support
- Replace OBD callbacks with flags for calling generic callback
- Add an example image and icon to CN xdd
- Add function to SDO/UDP and socketwrapper for ARP query
- Add simulation stub and simulation interface
- Add porting guide
- Add API functions for process data exchange

### Fixes:
- Several fixes to silence static code analyzer
- Avoid deadlock with asynchronous auto-response frames
- Fix 'occurred' typos
- Increase the i210 link up timeout
- Fix NULL pointer dereference during edrv_exit
- Fix wrong NMT state check in processFillTx()
- Solve NMT command truncation
- Add NMT_MAX_NODE_ID check for dllkframe PRes handling
- Solve dereferencing of a null pointer

### Changes:
- Remove Zynq ISE design
- Remove Zynq ARM no-OS support
- Move Object Dictionary from the stack to the application
- Move process image setup for CiA302-4 to the application
- Use a locally administered MAC address in embedded demos
- Allow to build several oplk kernel modules at once
- Split FeatureFlags definition from actual value calculation
- Implement buffer allocation lib to edrv-8255x
- openmac-nios2: use alt_irq_pending() only with IIC
- Update documentation to Doxygen version 1.8.11
- Simplify assert function
- Add a general definition for assertions
- Reduce default debug level for all stack libraries
- Pass user timer arguments by reference
- Add pcap immediate mode support
- Add openMAC IP core Vivado port
- Remove the now unused oplk_cbGenericObdAccess() function
- Remove redundancies in errhndkcal implementations
- Cleanup error handler kernel include paths in drivers
- Add an update guide for users of previous stack versions
- Improve code quality in stack, contrib, driver and app modules
- Several code cleanups

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.4.1 {#sect_revision_v2_4_1}

This is the latest release of the V2.4 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.4.1. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Fix several compiler warnings
- Increase isochrRxMaxPayload limit
- Remove Veth code from edrv if Veth support is disabled
- Add findDebugfs() prototype
- Fix openCONFORMANCE warnings in testcase 3-2-9-T2
- Remove sync interrupt registration for Nios II in dualprocshm
- Use Xil_L1DCache*() macros for Microblaze cache handling
- Fix the build when no PIO are used
- daemon PCP nios2: remove unused header
- Add missing comment for object event forwarding
- Fix relict in padding area of asynchronous frames
- Fix cirbuffer return values
- Connect atomic ipcore directly to shared memory
- Fix circbuf locking issues with host interface CAL
- Fix segmented read access to user specific OD
- Fix various typos in ctrlu files
- Enable storage attribute for PDO comm and mapp param objects
- Force 0x1F8D/F0 object in Demo_3CN openCONFIGURATOR project
- veth: avoid kernel header issue with musl
- SDO read fails for VString objects
- apps: include stdarg.h in eventlogstring.h
- PCIe Driver: Disable Werror=date-time for kernel >= 3.14
- Fix name of sched_priority element
- oplkcfg.h: Remove option CONFIG_OBD_USE_LOAD_CONCISEDCF

### Additions:
- Add XDD for CiA401 slave with configuration store/restore
- Improve user OD access comment

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.4.0 {#sect_revision_v2_4_0}

This is the first release of the V2.4 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.3 series (\ref sect_revision_v2_3_2)

Following is a summary of changes in V2.4.0. For a detailed revision history
refer to the Git source code history.

### New Features:
- Add redundancy support for edrv-i210
- Add dev_name command line parameter to console demos
- Handle NMT commands in kernel layer
- Add buffer allocation lib for edrv (8111, 8139, i210 and 82573)
- Add support for driver firmware update in Linux PCIe design
- Enhance openMAC Tx and Rx descriptor count
- Add dynamic sync interrupt adjustment in openMAC edrvcyclic
- Add hrestimer function to set timer with absolute time
- Enable exception stack for Nios II MN driver
- Add tightly-coupled data master to MN PCP
- Add tightly-coupled memory size parameter to subsystems
- Add RMN openCONFIGURATOR demo project

### Fixes:
- Fix typos in documentation
- Fix doubled synchronization timer IRQ
- Prevent multiple call to string object callback
- Correct OD callbacks for restore objects
- Improve SDO sequence layer initialization
- Adapt sequence layer timeout to EPSG DSP 301 V1.2.0
- Prevent overtaking SDO history frames
- Improve transmission time for segmented SDO transfer
- Fix C5 SoC host boot

### Changes:
- Add support for VEth interface in Windows PCIe solution
- MN Objdict: add new dynamic channel for REAL PDO objects
- Remove staging directory
- Rework DE2-115 board designs
- Replace clock crossing by pipeline bridge
- Implement second TX queue in edrv-i210
- Add generic time triggered flag to TX buffer structure
- Change image of devboard terasic-de2-115
- Add OD init parameters to stack API init parameters
- Export the stack-internal OD callback function via the API
- Rename OD user module to obdu
- Split OD header into API and user part
- Handle PDO mapping object access in the generic OD callback
- Handle error handler object access in the generic OD callback
- Handle CFM object access in the generic OD callback
- Refactor OD callback in ctrlu module
- Separation of SDO and OD
- Add interface for user specific OD for non-existing objects handling
- Refactor SDO sequence layer
- Add documentation for parallel interface ipcore
- Introduce board specific configuration header
- Introduce Rx buffer configuration parameter
- Enable coexistence of openMAC TTTX and auto response
- Remove openMAC "no filter match" IRQ
- Rename tightly-coupled memory
- Reduce FPGA MN event queue sizes
- Add AMI getter to target section header

### Known Issues:

Refer to the [known issues](\ref page_known_issues).

## V2.3.2 {#sect_revision_v2_3_2}

This is the latest release of the V2.3 release series. This release is a stable
release, it contains fixes and optimizations.

For a detailed revision history refer to the Git source code history.

### Known Issues:
- Linux i210 (see 2.3.1)
- DE2i-150 board (see 2.3.0)
- Windows PCIe (see 2.3.0)
- Linux PCIe (see 2.3.0)
- NDIS intermediate driver (see 2.3.0)
- Redundancy MN (see 2.2.1)

## V2.3.1 {#sect_revision_v2_3_1}

This is the latest release of the V2.3 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.3.1. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Fix Tx errors on Altera C5 SoC board
- Fix NMT request frame size
- Fix startup domain objects linking
- Fix Zynq Linux MN wrong cycle time issue for large cycle times
- Fix the distribution of the network configuration
- Prevent race in DLL when running as MN
- Fix startup problem with CNs using PRes Chaining
- Fix full kernel-internal queue on FPGA MN targets
- Call openMAC IRQ handler before handling 2nd Tx queue
- Several fixes in xdd and OD
- Fix Altera EPCS programming target
- Fix filter generation issues for Windows PCIe driver
- Fix line endings for Windows specific files
- Fix Linux i210 CN configuration error
- Fix different NMT state in PRes and Ident/StatusResponse

### Additions:
- Rework user stack event handling for Linux and Windows PCIe solutions
- Add troubleshooting information for CRC error on Altera C5 SoC
- Extend the maximum cycle time operation for Zynq Linux MN
- Mitigate feature check at stack start
- Cleanup stack cmake files
- Enable EPCS in DE2-115 CN dual design for PCP
- Enable virtual Ethernet feature in dualprocshm platforms

### Known Issues:
- Linux i210:
    - The Linux kernel space module with i210 must be compiled for CN explicitly
      if a CN application is used. If the kernel space module is compiled as MN,
      the application must initialize the stack as MN (node ID 0xF0).
- DE2i-150 board (see 2.3.0)
- Windows PCIe (see 2.3.0)
- Linux PCIe (see 2.3.0)
- NDIS intermediate driver (see 2.3.0)
- Xilinx Zynq MN (see 2.1.0)
- Redundancy MN (see 2.2.1)

## V2.3.0 {#sect_revision_v2_3_0}

This is the first release of the V2.3 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.2 series (\ref sect_revision_v2_2_2)

Following is a summary of changes in V2.3.0. For a detailed revision history
refer to the Git source code history.

### New Features:
- Introduce timesync module (replacing PDO sync implementation)
- Add application synchronization period configuration
- Add SoC time forwarding to application
- Add store restore feature for OS based CN
- Improve display of LEDs in demo_mn_qt
- Optimize the Altera FPGA CN performance
- Add TERASIC DE2i-150 MN design
- Add Windows NDIS intermediate driver
- Add Linux PCIe solution for B&R APC/PPC2100 and TERASIC DE2i-150

### Fixes:
- Fix SDO command layer dependencies
- Fix compiler warnings
- Avoid error in embedded demos if virtual Ethernet is disabled
- Fix application hangs with Windows kernel drivers
- Fix crashes happening during shutdown

### Changes:
- Improve SDO sequence layer segmented transfer
- Update host interface IP-Core description
- Rework openMAC second timer resources
- Revise Zynq MN address map and remove system.xml workaround
- Rework SDO over UDP module
- Export benchmark pins to FMC2 on ZC702 demo

### Known Issues:
- DE2i-150 board:
    - Programming the firmware onto the serial Flash device is not supported. Use
      JTAG to download the FPGA bitstream and Nios II elf manually.
- Windows PCIe:
    - Windows network device actions such as disable, restart and enable network
      adapter are not supported.
- Linux PCIe:
    - During kernel module insertion driver signing warnings are raised on systems
      with CONFIG_MODULE_SIG enabled in the Linux kernel configuration. The
      warnings can be ignored.
    - High CPU load could be caused because of low POWERLINK cycle times and
      high PDO sizes. Use the application synchronization period configuration
      (\ref tOplkApiInitParam::minSyncTime) to reduce the frequency of
      synchronization processing.
- NDIS intermediate driver:
    - In a system with multiple network adapters it is not possible to select
      the adapter to be used. The driver is configured to automatically select
      the first driver for network communication.
- Xilinx Zynq MN (see 2.1.0)
- Redundancy MN (see 2.2.1)

## V2.3.0-rc1 {#sect_revision_v2_3_0_rc1}

This is the release candidate for the upcoming 2.3.0 release.

Following is a summary of changes in V2.3.0-rc1. For a detailed revision history
refer to the Git source code history.

### New Features:
- Add late release support to dualprocshm
- Add dual processor CN designs for TERASIC DE2-115 board
- Add IP-Core to enable atomic exchange on Altera Nios II
- Add Windows PCIe solution for B&R APC/PPC2100 (Platform with Altera Cyclone 4
  FPGA and Intel Atom)
- Enhance debugstr to print NMT node commands
- Add eventlog library
- Add file transfer capability

### Fixes:
- Exclude objects if IP is not supported

### Changes:
- Update stack to support heterogeneous systems (64 bit support)
- Rework stack initialization
- Move LED module to kernel layer
- Rework kernel event handling for FPGA drivers
- Use Peterson's lock for Windows PCIe solution

### Known Issues:
- Xilinx Zynq MN (see 2.1.0)
- Redundancy MN (see 2.2.1)

### Outlook:
This release candidate is not feature complete. The following features are
expected to be released with the final 2.3.0 revision:
- Forward SoC time stamp to application layer
- Configuration storage (generic implementation)
- NDIS intermediate driver for Windows
- Linux PCIe solution for B&R APC/PPC2100
- Virtual Ethernet for PCIe solution
- MN demo design on TERASIC DE2i-150 board

## V2.2.2 {#sect_revision_v2_2_2}

This is the latest release of the V2.2 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.2.2. For a detailed revision history
refer to the Git source code history.

### Fixes:
- timer-linuxuser: Avoid segfault on oplk_init() failure
- demo_mn_console: Improve error handling and fix compiler warnings
- Free process image before oplk_shutdown()
- Fix DE2-115 SRAM timing issues
- Warn if ARCH or CROSS_COMPILE are not set when cross-compiling
- Disable Werror=date-time for kernel >= 3.14
- Fix CFM feature mismatch between demo and stack
- Remove RPDO update rejection in READY_TO_OPERATE state
- Fix doxygen warnings
- eventkcal-linuxkernel: Use cpumask APIv2
- Repair SDO sequence layer histroy buffer
- Avoid corrupted print for embedded cn example
- Reduce timeout steps in ctrlucal_executeCmd
- Fix wrong data cache flush in pdoucal-triplebufshm.c
- Correct error check for incompatible mapping
- Set interrupt source to edge-triggered for Zynq ARM
- Fix Linux kernel module loading error when using plkload

### Changes:
- Remove configuration option CONFIG_OBD_USE_LOAD_CONCISEDCF
- Acknowledge openMAC synctimer at beginning of ISR

### Known Issues:
- Xilinx Zynq MN (see 2.1.0)
- The redundancy MN does not work with the Intel i210 Linux kernel driver.

## V2.2.1 {#sect_revision_v2_2_1}

This is the latest release of the V2.2 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.2.1. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Fix C5 SoC misses synchronization interrupts
- Fix C5 SoC boot issue
- Fix asynchronous Rx count in DLLKCAL
- Fix openMAC Ethernet driver cache handling
- Fix compilation on Linux kernels > 3.17
- Fix CMake configuration errors for applications on windows
- Fix mnobd.cdc for redundancy demo Demo_RMN_3CN
- Several code cleanups and fixes

### Additions:
- Add guidance to fix Xilinx standalone BSP for Zynq ARM
- Add additional PCI ID for Intel I210 derivative

### Known Issues:
- Xilinx Zynq MN (see 2.1.0)
- The redundancy MN does not work with the Intel i210 Linux kernel driver.

## V2.2.0 {#sect_revision_v2_2_0}

This is the first release of the V2.2 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.1 series (\ref sect_revision_v2_1_2)

Following is a summary of changes in V2.2.0. For a detailed revision history
refer to the Git source code history.

### New Features:
- POWERLINK Master redundancy according to EPSG DS 302-A
- New platform Altera C5 SoC
- Mapping of more than 254 PDOs per node (multiple PDO channels)
- New dialog for SDO transfers in demo_mn_qt
- New dialog to execute local NMT commands in demo_mn_qt

### Fixes:
- Fix shutdown crash in RealTek 8111
- Fix openMAC descriptor DPRAM conflicts
- Several code cleanups and fixes

### Changes:
- Rework openMAC timer register map
- Optimize openMAC target modules
- Reduce Nios II BSP memory footprint
- Rework Altera Nios II tools
- Add selection of reset vector in MN PCP subsystem
- Revise EPCS Flash programming
- Revise Altera Qsys subsystems
- Rework time triggered Tx config of openMAC edrv
- Rework Quartus project makefiles
- Add board specific cflags to Nios II stack libraries
- Add heap size limit in Nios II tools
- Rework type definitions used by user and kernel layer to be cross compatible
  between different architectures

### Known Issues:
- Xilinx Zynq MN (see 2.1.0)
- Altera C5 SoC MN
    - The C5 SoC sometimes fails to boot at power-on successfully. In order to
      force a successful start-up press the "WARM RESETn" button (S8) on the
      Altera C5 SoC development board.
      This re-triggers configuration of the FPGA and download of the Nios II elf.

## V2.1.2 {#sect_revision_v2_1_2}

This is the latest release of the V2.1 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.1.2. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Fix deferred Rx buffer macro name
- Limit async scheduler by deferred Rx buffer
- Fix MN assigns unrequested async slots
- Fix CiA401 CN XDD conformance issue
- Fix the maximum interrupt id check in dualprocshm
- Avoid null pointer call in hostifIrqSyncCb()
- Fix activity LED generation for openMAC
- Fix  pcap driver crash in demo apps
- Discard invalid PDOs
- Fix check of multiple bits in nmtmnu.c
- Process NMT command data in received NMT request frames
- Fix payload of first segment of WriteByIndex
- Reset fDoStore in CFM NodeInfo structure if not required
- Initialize and reset error for CN progress call back in CFM
- Use correct node linked list on deletion of local node
- Fix possible use after free of Tx buffer list in dllknode.c
- Use IRQ-aware spinlocks in circbuf-linuxkernel.c
- Unlink Linux semaphores before creating new ones
- Mask real-time signals in QT demo main() before QApplication
- Add fixes in timer-linuxuser

- Several source code and documentation cleanups

### Additions:
- Add new AInv event to debugstr.c

### Changes:
- Store sync response callback functions queue-based
- Improve object related debug print
- Change demo_mn_qt to show argument on errors from OBD module
- Add improvements for edrv8111
- Change stack and app CMake files to work when called by a top-level CMake file

### Known Issues:
- Xilinx Zynq MN (see 2.1.0)

## V2.1.1 {#sect_revision_v2_1_1}

This is the latest release of the V2.1 release series. This release is a stable
release, it contains fixes and optimizations.

Following is a summary of changes in V2.1.1. For a detailed revision history
refer to the Git source code history.

### Fixes:
- Fix segmentation fault in sdocom.c
- Fix NULL pointer crash in edrvcyclic
- Fix stack shutdown which can cause crashes in Linux kernel module
- Fix Linux mutex initialization
- Fix I210 driver initialization
- Fix MN assignment of unrequested asynchronous slots
- Fix update of StatusResponse TX buffer
- Fix uninitialized function pointer access in dllcal
- Fix wrong string compare for emacps driver in CMakeLists.txt
- Remove declaration of non-existing ledu_addInstance() function
- Uncomment prescaler object in CiA302-4_MN XDD
- Change number of tErrHistoryEntry elements
- Cosmetic code cleanups
- Fix wrong parameter documentation

### Additions:
- Add documentation for POWERLINK frame structures
- Add improved error output in demos

### Removals:
- Remove unused openCONFIGURATOR projects

### Changes:
- Upgrade openCONFIGURATOR projects for openCONFIGURATOR 1.4.1
- Increase number of TX buffers in Ethernet drivers
- Enable deferred release for Linux Ethernet drivers
- Optimize initialization values of WaitSocPreq_U32 in demos
- Restructure hardware build documentation
- Improve documentation

### Known Issues:
- Xilinx Zynq MN (see 2.1.0)


## V2.1.0 {#sect_revision_v2_1_0}

This is the first release of the V2.1 release series. It contains new features
and functionalities. For productive environments it is recommended to use the
latest stable release of the V2.0 series (\ref sect_revision_v2_0_2)

Following is a summary of changes in V2.1.0. For a detailed revision history
refer to the Git source code history.

### New features:
- Xilinx Zynq support for No-OS systems
- Support Linux on Xilinx Zynq ARM
- Linux Ethernet driver for Realtek-8111/8168
- Support for detecting application/driver compatibility

### Fixes:
- Fix openMAC address decoder configuration
- Fix clock crossing IP-Core
- Fix MN issuing StatusRequest after NMTEnableReadyToOperate
- Fix Add missing interrupt dummy parameters for PCP
- Fix doxygen issues
- Fix compiler warnings
- Fix potential crash in PRes forwarding
- Fix crashes on PDO copy
- Fix potential NULL pointer access in process image functions
- Connect sysid to host in dual processor design

### Additions:
- Add default gateway handling in embedded demos
- Add Xilinx Zynq MN design for Z702 board
- Add UART redirection module for ZC702 board
- Add Zynq MN demo with Microblaze and openMAC in FPGA-part (PL)
- Add Zynq FSBL CMake project
- Add SD FAT16 file system library
- Add high resolution timer module for Xilinx Zynq
- Add Edrv module for Xilinx Zanq Gigabit Ethernet controller
- Add cross toolchain file for Xilinx ARM
- Using hardware divider on Nios II MN to improve performance
- Add a kernel internal queue for fill Tx events on No-os systems
- Add hostinterface IP-Core for Xilinx designs
- Add parallel bus master IP-Core for Altera designs
- Add Microblaze support in host interface driver
- Add dualprocshm library for No-OS systems
- Add architecture specific mutex implementation

### Changes:
- Improve documentation
- Miscellaneous code cleanups
- Cleanly split header files into public and private files
- Change host interface to use circular buffer library
- Update IP-Cores to version 1.0.2
- Update Altera Quartus and Qsys demos
- SDRAM timing constraints are added in INK demos
- Add push buttons to TERASIC MN demo designs
- Enhancements in demo_mn_embedded
- Optimize kernel PDO module

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

## V2.0.2 {#sect_revision_v2_0_2}

This is the final release of openPOWERLINK V2.0.2. This release is a stable
release, it contains fixes and optimizations.

### Fixes:
- Add missing CMake option for sync thread in demo_cn_console
- Correctly stop synchronous data thread in demo apps
- Prevent API to call stack functions if stack is not ready
- Fix demo_cn_console segmentation fault
- Fix resending of the same Tx buffer in BasicEthernet state
- Fix MN asynchronous slot self-invitation
- Fix shutdown behaviour if kernel initialization fails
- Add missing definitions in debugstr.c
- Fix 8255x Edrv NULL pointer crash
- Fix access rights in obdmacro.h
- Fix typos in 00000000_POWERLINK_CiA302-4_MN.xdd
- Fix initialization of process image
- Fix MN issuing StatusRequest after NMTEnableReadyToOperate
- Add missing openMAC waveforms to documentation
- Resolve Xilinx cmake filename case issue
- Reimplement handling of PRes in transition DLL_CT4
- Fix plkload to correctly handle multiple threads/irqs
- Fix dereferencing warning in pdo CAL
- Fix compiler warnings in host interface SW modules and openMAC drivers
- Fix used interrupt api for Altera IP-Cores
- Fix SDO busy intern after CN reset command

### Changes:
- Extend PDO mapping subindexes
- Rename RMII Rx data valid input in openMAC IP-Core

### Additions:
- Add additional documention about NOTFOUND CMake error


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

