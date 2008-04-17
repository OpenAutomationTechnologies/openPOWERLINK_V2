
#include "global.h"
#include   "fec.h"
#include   "ks8721.h"
#if TARGET_SYSTEM == _NO_OS_
    #include   "common.h"

#elif TARGET_SYSTEM == _LINUX_

    #include <linux/config.h>
    #ifdef CONFIG_COLDFIRE
        #include <asm/coldfire.h>
    #endif

#else
    #error "TARGET_SYSTEM currently not supported by FEC driver"
#endif
//#include   "error.h"
//#include   "arch.h"



/************************************************************************
* NAME: ks8721_init_transceiver
*
* DESCRIPTION: This function initializes the transceiver
*
* RETURNS: If no error occurs, this function returns OK.
*************************************************************************/
int ks8721_init_transceiver(unsigned long base_addr, int *fduplex)
{

//    unsigned long time;
    unsigned long data;
//    int flag = 0;

    // Set the frequency of the MII
    FEC_MSCR(base_addr) = FEC_MII_SPEED;

    // Reset
    if (fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_RESET))
    {
        return ERR;
    }

    // Read back
    if (fec_read_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, (unsigned int*) &data))
    {
        return ERR;
    }

    // If reset bit is set, return
    if (data & KS8721_CTRL_RESET)
    {
        return ERR;
    }

// we set the phy to half duplex and 100BaseT
// full duplex is not allowed in EPL

/*
    // Disable  the auto-negotiation
    if (fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, 0))
       return ERR;

    // Set the auto-negotiation advertisement register
    if (fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_ANADV, KS8721_ANADV_ADV_ALL))
       return ERR;

    // Enable the auto-negotiation
    if (fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_AN_ENABLE))
       return ERR;

    // Read PHY status register
    if (fec_read_mii(base_addr, FEC_PHY_ADDR, KS8721_STAT, (unsigned int*) &data))
       return ERR;

    time = current_time;
    // Wait for the auto-negotiation completion
    while(!(data & KS8721_STAT_ANCOMPLETE))
    {
       // Read PHY status register
       if (fec_read_mii(base_addr, FEC_PHY_ADDR, KS8721_STAT, (unsigned int*) &data))
          return ERR;
       if(timer_get_interval(time, current_time) > KS8721_AUTONEG_TIMEOUT)
       {
          flag = 1;
          break;
       }
    }

    // Set the duplex flag
    if(!flag)
    {
      if(data & KS8721_STAT_FDUPLEX)
         *fduplex = 1;
      else
         *fduplex = 0;

       return OK;
    }
*/
    // Set the default mode (half duplex, 100 Mbps)
    if (fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_DEFAULT_MODE_100))
    // Set the default mode (half duplex, 10 Mbps)
//    if (fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_DEFAULT_MODE_10))
    {
        return ERR;
    }
    *fduplex = 0;

    return OK;

}
