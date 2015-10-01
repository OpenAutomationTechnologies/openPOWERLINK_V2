Socket wrapper for SDO/UDP
==========================

## Introduction

The SDO over UDP capability requires IP/UDP supported by the target platform.
On OS platforms (e.g. Linux) a socket is created for the POWERLINK node's UDP
connection. Other platforms, which have no socket interface support, the socket
wrapper is used for interfacing to customized IP or similar protocol stacks.

## Requirements

If there is no SDO/UDP target-specific implementation available for your target
platform, it is recommended to use the socket wrapper interface.

In the following sections the functions to be implemented for the target platform
are discussed.

### Create

The `socketwrapper_create` function is called by the SDO/UDP module to create
the socket wrapper instance. The caller provides the socket receive callback
function pointer, which is called if a frame is received from the socket.
The function must return a pointer to the successfully created socket wrapper
instance, otherwise `SOCKETWRAPPER_INVALID` has to be returned.

### Bind

The `socketwrapper_bind` function configures the created socket wrapper by setting
the local IP address and port.
If the function fails, it shall return `kErrorSdoUdpNoSocket`.

### Close

The `socketwrapper_close` function is called to close the connection. Any frame
that is still received after closing shall be dropped, also frame transmissions
with `socketwrapper_send` shall be rejected.
If the function fails, it shall return `kErrorSdoUdpSocketError`.

### Send

The `socketwrapper_send` function is used to transmit a packet to the UDP connection.
The caller must specify the remote IP address and port with using `pRemote_p`.
The arguments `pData_p` and `dataSize_p` reflect the UDP payload only.
If the function fails, it shall return `kErrorSdoUdpSendError`.

### Critical section

The `sdoudp_criticalSection` function is required to be implemented if the
receive callback is called from a different thread context. Otherwise, the
function can be left empty.

### Receive callback

The socket receive callback `receiveFromSocket` (`sdoudp-socketwrapper.c`) shall
be called when a frame is received from the network that matches the binding
configurations (IP address and port). The arguments `pData_p` and `dataSize_p`
reflect the UDP payload of the received frame. The `pRemote_p` argument gives the
remote IP address and port.
