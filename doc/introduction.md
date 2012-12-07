Introduction {#intro}
============

## Ethernet POWERLINK
Ethernet POWERLINK is a Real-Time Ethernet field bus system. It is
based on the Fast Ethernet Standard IEEE 802.3.

A managing node (MN), which acts as the master in the POWERLINK network,
polls the controlled nodes (CN) cyclically. This process takes place in
the isochronous phase of the POWERLINK cycle. Immediately after the
isochronous phase an asynchronous phase for communication follows
which is not time-critical, e.g. TCP/IP communication. The isochronous
phase starts with the Start of Cyclic (SoC) frame on which all
nodes are synchronized. This schedule design avoids collisions which
are usually present on standard Ethernet, and ensures the determinism
of the hard real-time communication. It is implemented in the POWERLINK
data link layer. The EPL network can be connected via gateways to
non real-time networks.

The communication profile of Ethernet POWERLINK is adapted from
CANopen. Thus, design principles such as process data object (PDO)
for the exchange of process variables and service data object (SDO)
for the configuration of remote object dictionaries  are reused. All
PDOs are exchanged within the isochronous phase, similar to the
synchronous PDOs of CANopen. This is because event-triggered
PDOs would interfere with hard real-time requirements.

To be conforming to IEEE 802.3, each POWERLINK device has a
unique MAC address. Additionally, each device is assigned a logical
node ID. Mostly, this node ID can be configured via node switches on
the device. If a particular EPL device implements a TCP/IP stack, it
gets a private IP address from class C within the network
192.168.100.0 where the host part equals the POWERLINK node ID.

It is assumed that you are familiar with the Ethernet POWERLINK
Communication Profile Specification.  

## openPOWERLINK Key Features
- Implements Communication profile EPSG 301 1.1.0
- Data link layer and NMT state machine for Controlled and Managing Nodes
- Configuration Manager for configuration of CNs at run-time
- SDO via UDP and POWERLINK ASnd frames
- Dynamic PDO mapping
- User-configurable object dictionary
- Supports the POWERLINK cycle features async-only CN and multiplexed CN
- Supports the POWERLINK extension PollResponse Chaining according to
  EPSG 302-C V 0.0.3 on MN and CN
- Implemented in plain ANSI C
- Modular software structure for simple portability to different target
  platforms
- Supports target platforms with and without operating system
- Event-driven Communication Abstraction Layer
- Provides Generic API for user-application

## Supported Platforms
- [VxWorks](\ref vxworks)
- [Linux X86](\ref linux-x86)
- [Windows](\ref other-platforms)
- [Altera NiosII](\ref altera)
- [Xilinx Microblaze](\ref xilinx)
- [Freescale Coldfire MCF5484](\ref other-platforms)
- [Hilscher netX-500](\ref other-platforms)
- [Atmel AT91RM9200 with Davicom DM9003 under Linux](\ref other-platforms)

