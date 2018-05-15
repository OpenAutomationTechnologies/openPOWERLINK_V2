/**
********************************************************************************
\file   veth-linuxdpshm.c

\brief  Implementation of virtual Ethernet for Linux PCIe interface

This file contains the the virtual Ethernet driver for the Linux PCIe
interface implementation.

\ingroup module_veth
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <drvintf.h>

#if defined(CONFIG_INCLUDE_VETH)
#include <net/arp.h>
#include <net/protocol.h>
#include <net/pkt_sched.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/skbuff.h>  /* for struct sk_buff */

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef VETH_TX_TIMEOUT
#define VETH_TX_TIMEOUT 0       // d.k.: we use no timeout
#endif

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static struct net_device* pVEthNetDevice_g = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int                      vethOpen(struct net_device* pNetDevice_p);
static int                      vethStop(struct net_device* pNetDevice_p);
static int                      vethStartXmit(struct sk_buff* pSkb_p,
                                              struct net_device* pNetDevice_p);
static struct net_device_stats* vethGetStats(struct net_device* pNetDevice_p);
static void                     vethTxTimeout(struct net_device* pNetDevice_p);
static tOplkError               receiveFrameCb(const tFrameInfo* pFrameInfo_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static const struct net_device_ops oplk_netdev_ops =
{
    .ndo_open               = vethOpen,
    .ndo_stop               = vethStop,
    .ndo_get_stats          = vethGetStats,
    .ndo_start_xmit         = vethStartXmit,
    .ndo_tx_timeout         = vethTxTimeout,
    .ndo_change_mtu         = eth_change_mtu,
    .ndo_set_mac_address    = eth_mac_addr,
    .ndo_validate_addr      = eth_validate_addr,
};

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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))
    // Allocate net device structure with priv pointing to stats structure
    pVEthNetDevice_g = alloc_netdev(sizeof(struct net_device_stats),
                                    PLK_VETH_NAME,
                                    ether_setup);
#else
    // Allocate net device structure with priv pointing to stats structure
    pVEthNetDevice_g = alloc_netdev(sizeof(struct net_device_stats),
                                    PLK_VETH_NAME,
                                    NET_NAME_UNKNOWN, ether_setup);
#endif

    if (pVEthNetDevice_g == NULL)
        return kErrorNoResource;

    pVEthNetDevice_g->netdev_ops = &oplk_netdev_ops;
    pVEthNetDevice_g->watchdog_timeo = VETH_TX_TIMEOUT;
    pVEthNetDevice_g->destructor = free_netdev;

    // Copy own MAC address to net device structure
    OPLK_MEMCPY(pVEthNetDevice_g->dev_addr, aSrcMac_p, 6);

    // Register VEth to the network subsystem
    if (register_netdev(pVEthNetDevice_g))
    {
        DEBUG_LVL_VETH_TRACE("%s(): Could not register VEth...\n", __func__);
    }
    else
    {
        DEBUG_LVL_VETH_TRACE("%s(): Register VEth successful...\n", __func__);
    }

    return kErrorOk;
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
    if (pVEthNetDevice_g != NULL)
    {
        // Unregister VEth from the network subsystem
        unregister_netdev(pVEthNetDevice_g);

        // Destructor was set to free_netdev,
        // So we do not need to call free_netdev here
        pVEthNetDevice_g = NULL;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Open entry point of virtual Ethernet driver

The function contains the open routine of the virtual Ethernet driver.

\param[in,out]  pNetDevice_p        Pointer to net device structure.

\return The function returns an error code.
*/
//------------------------------------------------------------------------------
static int vethOpen(struct net_device* pNetDevice_p)
{
    tOplkError  ret;

    // Open the device
    // Start the interface queue for the network subsystem
    netif_start_queue(pNetDevice_p);

    // Register callback function in interface driver
    ret = drvintf_regVethHandler(receiveFrameCb);
    DEBUG_LVL_VETH_TRACE("%s(): drvintf_regVethHandler returned 0x%04X\n", __func__, ret);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Close entry point of virtual Ethernet driver

The function contains the close routine of the virtual Ethernet driver.

\param[in,out]  pNetDevice_p        Pointer to net device structure.

\return The function returns an error code.
*/
//------------------------------------------------------------------------------
static int vethStop(struct net_device* pNetDevice_p)
{
    DEBUG_LVL_VETH_TRACE("%s()\n", __func__);

    drvintf_regVethHandler(NULL);
    netif_stop_queue(pNetDevice_p);     // Stop the interface queue for the network subsystem

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Transmit entry point of virtual Ethernet driver

The function contains the transmit function for the virtual Ethernet driver.

\param[in,out]  pSkb_p              Pointer to packet which should be transmitted.
\param[in,out]  pNetDevice_p        Pointer to net device structure of interface.

\return The function returns an error code.
*/
//------------------------------------------------------------------------------
static int vethStartXmit(struct sk_buff* pSkb_p, struct net_device* pNetDevice_p)
{
    tOplkError      ret;
    tFrameInfo      frameInfo;

    // Transmit function
    struct net_device_stats* pStats = netdev_priv(pNetDevice_p);

    // Save time stamp
    pNetDevice_p->trans_start = jiffies;

    frameInfo.frame.pBuffer = (tPlkFrame*)pSkb_p->data;
    frameInfo.frameSize = pSkb_p->len;

    // Call send function on interface driver
    ret = drvintf_sendVethFrame(&frameInfo);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_VETH_TRACE("%s(): drvintf_sendVethFrame returned 0x%04X\n", __func__, ret);
        netif_stop_queue(pNetDevice_p);
    }
    else
    {
        DEBUG_LVL_VETH_TRACE("%s(): frame passed to interface driver\n", __func__);
        dev_kfree_skb(pSkb_p);

        // Set stats for the device
        pStats->tx_packets++;
        pStats->tx_bytes += frameInfo.frameSize;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get statistics

The function gets the statistics of the interface.

\param[in,out]  pNetDevice_p        Pointer to net device structure of interface.

\return The function returns a pointer to a net_device_stats structure.
*/
//------------------------------------------------------------------------------
static struct net_device_stats* vethGetStats(struct net_device* pNetDevice_p)
{
    DEBUG_LVL_VETH_TRACE("%s()\n", __func__);

    return netdev_priv(pNetDevice_p);
}

//------------------------------------------------------------------------------
/**
\brief  TX timeout entry point

The function provides the TX timeout entry point of the driver.

\param[in,out]  pNetDevice_p        Pointer to net device structure of interface.
*/
//------------------------------------------------------------------------------
static void vethTxTimeout(struct net_device* pNetDevice_p)
{
    DEBUG_LVL_VETH_TRACE("%s()\n", __func__);

    if (netif_queue_stopped(pNetDevice_p))
        netif_wake_queue(pNetDevice_p);
}

//------------------------------------------------------------------------------
/**
\brief  Receive frame from virtual Ethernet interface

The function receives a frame from the virtual Ethernet interface.

\param[in]      pFrameInfo_p        Pointer to frame information of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError receiveFrameCb(const tFrameInfo* pFrameInfo_p)
{
    tOplkError               ret = kErrorOk;
    struct net_device*       pNetDevice = pVEthNetDevice_g;
    struct net_device_stats* pStats = netdev_priv(pNetDevice);
    struct sk_buff*          pSkb;

    DEBUG_LVL_VETH_TRACE("%s(): FrameSize=%u\n", __func__, pFrameInfo_p->frameSize);

    if ((pSkb = dev_alloc_skb(pFrameInfo_p->frameSize + 2)) == NULL)
    {
        pStats->rx_dropped++;
        goto Exit;
    }
    pSkb->dev = pNetDevice;

    skb_reserve(pSkb, 2);

    OPLK_MEMCPY((void*)skb_put(pSkb, pFrameInfo_p->frameSize),
                 pFrameInfo_p->frame.pBuffer, pFrameInfo_p->frameSize);

    pSkb->protocol = eth_type_trans(pSkb, pNetDevice);
    pSkb->ip_summed = CHECKSUM_UNNECESSARY;

    netif_rx(pSkb);         // Call netif_rx with skb

    DEBUG_LVL_VETH_TRACE("%s(): SrcMAC: %02X:%02X:%02x:%02X:%02X:%02x\n",
                         __func__,
                         pFrameInfo_p->frame.pBuffer->aSrcMac[0],
                         pFrameInfo_p->frame.pBuffer->aSrcMac[1],
                         pFrameInfo_p->frame.pBuffer->aSrcMac[2],
                         pFrameInfo_p->frame.pBuffer->aSrcMac[3],
                         pFrameInfo_p->frame.pBuffer->aSrcMac[4],
                         pFrameInfo_p->frame.pBuffer->aSrcMac[5]);

    // Update receive statistics
    pStats->rx_packets++;
    pStats->rx_bytes += pFrameInfo_p->frameSize;

Exit:
    return ret;
}

/// \}

#endif // CONFIG_INCLUDE_VETH
