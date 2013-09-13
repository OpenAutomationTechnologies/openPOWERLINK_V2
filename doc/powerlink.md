Ethernet POWERLINK {#page_powerlink}
==================

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