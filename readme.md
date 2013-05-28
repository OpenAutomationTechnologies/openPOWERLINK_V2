README - Quick Start Guide {#readme}
==========================

    (c) SYSTEC electronic GmbH, August-Bebel-Str. 29, D-07973 Greiz,
        http://www.systec-electronic.com

    (c) Bernecker + Rainer Industrie Elektronik Ges.m.b.H., B&R Strasse 1, A-5142 Eggelsberg
        http://www.br-automation.com
        
    (c) Kalycito Infotech Private Limited

## Documentation

* The documentation of the openPOWERLINK protocol stack can be found in the
  subdirectory "doc". It is written in _markdown_ markup format.
* The openPOWERLINK software manual can be generated from the markdown
  documentation and the in source-code documentation with the tool doxygen.
  Therefore Doxygen version greater or equal 1.8 is required. The software manual
  will be created in HTML format under `doc/software-manual/html`.
  
  To generate the software manual:
  
      > cd doc/software-manual
      > doxygen

* Further documentation can be found on the sourceforge website:  
  <http://sourceforge.net/projects/openpowerlink/>
  
* License: Please refer to the file "license.txt" for information about
  the license of the source code.

## Generic Requirements for all demo applications

- POWERLINK Network:
  * one or more POWERLINK I/O CN devices according device profile CiA-401
  * Node-IDs of the Controlled Nodes (CN): 1, 32 or 110
  * CAT5 cables to connect the POWERLINK devices with the demo application

- openPOWERLINK demo application with node-ID 240/0xF0.

## Available demo applications

- [X86 PC with Linux](\ref linux-x86) or [Windows operating system](\ref other-platforms)
  * MN demo application using a console based interface\n
    = `examples/demo_mn_console`
  * CN demo application using a console based interface\n
    = `examples/demo_cn_console`
  * MN demo application using a Qt based interface\n
    = `examples/demo_mn_qt`

  These demo applications can be built for both Linux and Windows.
  CMake is used as a cross-platform build system.

  On Linux, these demo applications can be configured to use either
  a pcap based kernel stack that is located in user space, or to use a
  kernel stack that is located in kernel space.

  On Windows, these demo applications only support a pcap based stack in
  user space.

- [Altera Cyclone III/IV](\ref altera-cn \ref altera-mn) on EBV DBC3C40, SYS TEC ECUcore-EP3C or TERASIC DE2-115
  Development Board with Nios II Soft-CPU and POWERLINK IP-Core.
  * CN demo which controls the LEDs and reads the pushbuttons on the devboard:\n
    = `examples/arch/altera_nios2/no_os/gnu/demo_directIO`
  * MN demo\n
    = `examples/arch/altera_nios2/no_os/gnu/demo_mn`

- [Xilinx Spartan 6](\ref xilinx) on Industrial Ethernet Kit (IEK, LX150t) or MicroBoard (micro, LX9)
  with Microblaze Soft-CPU and POWERLINK IP-Core.
  * CN demo which controls the LEDs and reads the pushbuttons on the devboard:\n
    = `examples/arch/xilinx_microblaze/no_os/gnu/demo_cn_digitalio`

## Troubleshooting

* Make a trace with Wireshark on another PC that is connected to the POWERLINK
  network (www.wireshark.org)

