Ethernet POWERLINK {#page_powerlink}
==================

[TOC]

# Introduction {#sect_powerlink_introduction}

Ethernet POWERLINK is a Real-Time Ethernet field bus system. It is
based on the Fast Ethernet Standard IEEE 802.3.

A Managing Node (MN), acting as the master in the POWERLINK network,
polls the controlled nodes (CN) cyclically. This process takes place in
the isochronous phase of the POWERLINK cycle. Immediately after the
isochronous phase an asynchronous phase for communication follows
which is not time-critical, e.g. TCP/IP communication. The isochronous
phase starts with the Start of Cyclic (SoC) frame on which all
nodes are synchronized. This schedule design avoids collisions which
are usually present on standard Ethernet, and ensures the determinism
of the hard real-time communication. It is implemented in the POWERLINK
data link layer. The POWERLINK network can be connected via gateways to
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
the device. If a particular POWERLINK device implements a TCP/IP stack, it
gets a private IP address from class C within the network
192.168.100.0 where the host part equals the POWERLINK node ID.

It is assumed that you are familiar with the Ethernet POWERLINK
Communication Profile Specification.

For additional information on the Ethernet POWERLINK protocol refer to
the [Ethernet POWERLINK Standardisation Group EPSG](http://www.ethernet-powerlink.org).

# Object Dictionary (OD) {#sect_powerlink_od}

## Fundamentals {#sect_powerlink_od_fundamentials}

The Object Dictionary (OD) forms the essential connection between
the application software and the POWERLINK stack, which enables data to be
exchanged with an application over the POWERLINK network. POWERLINK defines the
services and communication objects for the access to the OD entries.

The OD of POWERLINK is modeled after the one of CANopen. Each entry is addressed
by index and sub index. The properties of an entry in the OD are defined by type
(UINT8, UIN16, REAL32, Visible String, Domain,...) and by attribute (read-only,
write-only, const, read-write, mappable).

The OD can contain up to 65536 index entries and 0 – 255 sub indexes per index.
They are predefined by communication profile or device profile. Type and attribute
for sub indexes within an index can vary. Entries can be preset with default
values. It is possible to modify the value of an entry with the help of SDOs
(Service Data Objects), in as far as it is allowed by the attribute
(read-write and write-only; not for read-only and const). A value can also be
modified by the application itself (attribute read-write, write-only and
read-only; not for const).

## Structure of an OD, Standardized Profiles {#sect_powerlink_od_structure}

The OD is divided into sections. The section 0x1000 – 0x1FFF is used to define
the parameters for the communication objects and for  storage of general information
(manufacturer, device type, serial number, etc.). The entries from index
0x2000 – 0x5FFF are reserved for the storage of manufacturer-specific entries.
The entries starting at 0x6000 are those with device specific objects as described
by the applicable device profile.

### Communication Profile

The Ethernet POWERLINK Specification defines the communication parameters for
the communication objects, which must be supported by every POWERLINK device.
In addition, for device-specific expansions to the communication profile applicable
CANopen device profiles may be used.

### Device Profiles
Overview of CANopen device profiles (not complete):
- Device profile for generic input/output modules (CiA 401)
- Device profile for drives and motion controls (CiA 402)
- Device profile for human/machine interface (HMI) (CiA 403)
- Device profile for measuring devices and closed-loop controllers (CiA 404)
- Device profile for encoders (CiA 406)
- Device profile for proportional valves (CiA 408)
For a complete overview of all currently available device profiles refer to:
http://www.can-cia.org/downloads/ciaspecifications/

