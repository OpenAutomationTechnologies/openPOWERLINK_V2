/**
********************************************************************************
\file   veth-linuxuser.c

\brief  Implementation of virtual Ethernet for Linux userspace

This file contains the the virtual Ethernet driver for the Linux userspace
implementation. It uses a TUN/TAP device as virtual Ethernet driver.

\ingroup module_veth
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/veth.h>
#include <kernel/dllk.h>
#include <kernel/dllkcal.h>

#if defined(CONFIG_INCLUDE_VETH)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TUN_DEV_NAME        "/dev/net/tun"

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing an instance of the Virtual Ethernet driver

This structure describes an instance of the Virtual Ethernet driver in Linux
userspace.
*/
typedef struct
{
    UINT8               macAdrs[6];         ///< MAC address of the VEth interface
    UINT8               tapMacAdrs[6];      ///< MAC address of the TAP device
    int                 fd;                 ///< File descriptor of the tunnel device
    BOOL                fStop;              ///< Flag indicating whether the receive thread shall be stopped
    pthread_t           threadHandle;       ///< Handle of the receive thread
} tVethInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tVethInstance        vethInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void       getMacAdrs(UINT8* pMac_p);
static tOplkError receiveFrameCb(tFrameInfo* pFrameInfo_p,
                                 tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static void*      vethRecvThread(void* pArg_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize virtual Ethernet

The function initializes the virtual Ethernet module.

\param[in]      aSrcMac_p           MAC address to set for virtual Ethernet interface.

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_init(const UINT8 aSrcMac_p[6])
{
    tOplkError      ret;
    struct ifreq    ifr;
    int             err;

    if ((vethInstance_l.fd = open(TUN_DEV_NAME, O_RDWR)) < 0)
    {
        DEBUG_LVL_VETH_TRACE("Error opening %s\n", TUN_DEV_NAME);
        return kErrorNoFreeInstance;
    }

    OPLK_MEMSET(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    strncpy(ifr.ifr_name, PLK_VETH_NAME, IFNAMSIZ);

    if ((err = ioctl(vethInstance_l.fd, TUNSETIFF, (void*)&ifr)) < 0)
    {
        DEBUG_LVL_VETH_TRACE("Error setting TUN IFF options\n");
        close(vethInstance_l.fd);
        return err;
    }

    // save MAC address of TAP device and Ethernet device to be able to
    // exchange them
    OPLK_MEMCPY(vethInstance_l.macAdrs, aSrcMac_p, 6);
    getMacAdrs(vethInstance_l.tapMacAdrs);

    // start tap receive thread
    vethInstance_l.fStop = FALSE;
    if (pthread_create(&vethInstance_l.threadHandle, NULL, vethRecvThread, (void*)&vethInstance_l) != 0)
        return kErrorNoFreeInstance;

#if (defined(__GLIBC__) && (__GLIBC__ >= 2) && (__GLIBC_MINOR__ >= 12))
    pthread_setname_np(vethInstance_l.threadHandle, "oplk-veth");
#endif

    // register callback function in DLL
    ret = dllk_regAsyncHandler(receiveFrameCb);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down virtual Ethernet

The function shuts down the virtual Ethernet module.

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_exit(void)
{
    tOplkError ret;

    // Unregister the receive callback function
    ret = dllk_deregAsyncHandler(receiveFrameCb);

    // stop receive thread by setting its stop flag
    vethInstance_l.fStop = TRUE;
    pthread_join(vethInstance_l.threadHandle, NULL);
    close(vethInstance_l.fd);

    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get MAC address of veth interface

The function reads the MAC address of the virtual Ethernet interface.

\param[out]     pMac_p              Pointer to store the MAC address
*/
//------------------------------------------------------------------------------
static void getMacAdrs(UINT8* pMac_p)
{
    struct ifreq    ifr;
    int             sock;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        DEBUG_LVL_VETH_TRACE("%s: Cannot open udp socket for MAC reading: %s\n",
                             __func__,
                             strerror(errno));
        return;
    }

    OPLK_MEMSET(&ifr, 0, sizeof(struct ifreq));
    strncpy(ifr.ifr_name, "plk", IFNAMSIZ);

    if (ioctl(sock, SIOCGIFHWADDR, &ifr) < 0)
    {
        DEBUG_LVL_VETH_TRACE("Cannot get MAC address: '%s' (%d)\n", strerror(errno), errno);
    }

    DEBUG_LVL_VETH_TRACE("Get Mac addr %02x:%02x:%02x:%02x:%02x:%02x\n",
                         ifr.ifr_hwaddr.sa_data[0],
                         ifr.ifr_hwaddr.sa_data[1],
                         ifr.ifr_hwaddr.sa_data[2],
                         ifr.ifr_hwaddr.sa_data[3],
                         ifr.ifr_hwaddr.sa_data[4],
                         ifr.ifr_hwaddr.sa_data[5]);

    close(sock);

    OPLK_MEMCPY(pMac_p, &ifr.ifr_hwaddr.sa_data[0], ETH_ALEN);
}

//------------------------------------------------------------------------------
/**
\brief  Receive frame from virtual Ethernet interface

The function receives a frame from the virtual Ethernet interface.

\param[in]      pFrameInfo_p        Pointer to frame information of received frame.
\param[out]     pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                                    set this flag to determine if the RxBuffer could be
                                    released immediately.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError receiveFrameCb(tFrameInfo* pFrameInfo_p,
                                 tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    UINT    nwrite;

    // replace the MAC address of the POWERLINK Ethernet interface with virtual
    // Ethernet MAC address before forwarding it into the virtual Ethernet interface
    if (OPLK_MEMCMP(pFrameInfo_p->frame.pBuffer->aDstMac, vethInstance_l.macAdrs, ETH_ALEN) == 0)
    {
        OPLK_MEMCPY(pFrameInfo_p->frame.pBuffer->aDstMac, vethInstance_l.tapMacAdrs, ETH_ALEN);
    }

    nwrite = write(vethInstance_l.fd, pFrameInfo_p->frame.pBuffer, pFrameInfo_p->frameSize);
    if (nwrite != pFrameInfo_p->frameSize)
    {
        DEBUG_LVL_VETH_TRACE("Error writing data to virtual Ethernet interface!\n");
    }

    *pReleaseRxBuffer_p = kEdrvReleaseRxBufferImmediately;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Receive frame from virtual Ethernet interface

The function receives a frame from the virtual Ethernet interface. It is implemented
to be used as a thread which does a blocking read in a while loop.

\param[in,out]  pArg_p              Thread argument. Pointer to virtual Ethernet instance.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static void* vethRecvThread(void* pArg_p)
{
    UINT8               buffer[ETH_DATA_LEN];
    UINT                nread;
    tFrameInfo          frameInfo;
    tOplkError          ret = kErrorOk;
    tVethInstance*      pInstance = (tVethInstance*)pArg_p;
    fd_set              readFds;
    int                 result;
    struct timeval      timeout;

    while (!pInstance->fStop)
    {
        timeout.tv_sec = 0;
        timeout.tv_usec = 400000;

        FD_ZERO(&readFds);
        FD_SET(pInstance->fd, &readFds);

        result = select(pInstance->fd + 1, &readFds, NULL, NULL, &timeout);
        switch (result)
        {
            case 0:     // timeout
                //DEBUG_LVL_VETH_TRACE("select timeout\n");
                break;

            case -1:    // error
                DEBUG_LVL_VETH_TRACE("select error: %s\n", strerror(errno));
                break;

            default:    // data from tun/tap ready for read
                nread = read(pInstance->fd, buffer, ETH_DATA_LEN);
                if (nread > 0)
                {
                    DEBUG_LVL_VETH_TRACE("VETH: Read %d bytes from the tap interface\n", nread);
                    DEBUG_LVL_VETH_TRACE("SRC MAC: %02X:%02X:%02x:%02X:%02X:%02x\n",
                                         buffer[6],
                                         buffer[7],
                                         buffer[8],
                                         buffer[9],
                                         buffer[10],
                                         buffer[11]);
                    DEBUG_LVL_VETH_TRACE("DST MAC: %02X:%02X:%02x:%02X:%02X:%02x\n",
                                         buffer[0],
                                         buffer[1],
                                         buffer[2],
                                         buffer[3],
                                         buffer[4],
                                         buffer[5]);
                    // replace src MAC address with MAC address of virtual Ethernet interface
                    OPLK_MEMCPY(&buffer[6], pInstance->macAdrs, ETH_ALEN);

                    frameInfo.frame.pBuffer = (tPlkFrame*)buffer;
                    frameInfo.frameSize = nread;
                    ret = dllkcal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioGeneric);
                    if (ret != kErrorOk)
                    {
                        DEBUG_LVL_VETH_TRACE("%s(): dllkcal_sendAsyncFrame returned 0x%04X\n", __func__, ret);
                    }
                }
                break;
        }
    }

    pthread_exit(NULL);

    return NULL;
}

/// \}

#endif // CONFIG_INCLUDE_VETH
