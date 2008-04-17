
#include "global.h"
#include   "fec.h"
#include   "ksz8893.h"
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
* NAME: ksz8893_init_transceiver
*
* DESCRIPTION: This function initializes the KSZ8893 to repeater mode.
*
* RETURNS: If no error occurs, this function returns OK.
*************************************************************************/
int ksz8893_init_transceiver(unsigned long base_addr, int *fduplex)
{

//    unsigned long time;
    unsigned int data;
//    int flag = 0;

    // Set the frequency of the MII
    FEC_MSCR(base_addr) = FEC_MII_SPEED;

    // d.k. read does not work at all, because FEC interprets OP code 00
    //      as write and not as read operation.
/*    // read chip family
    data = KSZ8893_READ(KSZ8893_CHIPID0);
    if (fec_access_mii(base_addr, &data))
    {
        return ERR;
    }

    if (KSZ8893_DATA(data) != KSZ8893_CHIPID0_VAL)
    {   // wrong chip
        return OK;
    }

    // read revision and switch status
    data = KSZ8893_READ(KSZ8893_CHIPID1);
    if (fec_access_mii(base_addr, &data))
    {
        return ERR;
    }

    if ((KSZ8893_DATA(data) & KSZ8893_CHIPID1_START) != 0)
    {   // switch already started
        return OK;
    }
*/
    // configure port 1
    data = KSZ8893_WRITE(KSZ8893_PORT1CTRL12, KSZ8893_PORTCTRL12_100);
    if (fec_access_mii(base_addr, &data))
    {
        return ERR;
    }

    // configure port 2
    data = KSZ8893_WRITE(KSZ8893_PORT2CTRL12, KSZ8893_PORTCTRL12_100);
    if (fec_access_mii(base_addr, &data))
    {
        return ERR;
    }

    // enable repeater mode and configure port 3
    data = KSZ8893_WRITE(KSZ8893_GLOBALCTRL4, (KSZ8893_GLOBALCTRL4_REP | KSZ8893_GLOBALCTRL4_HALF));
    if (fec_access_mii(base_addr, &data))
    {
        return ERR;
    }

    // start switch
    data = KSZ8893_WRITE(KSZ8893_CHIPID1, KSZ8893_CHIPID1_START);
    if (fec_access_mii(base_addr, &data))
    {
        return ERR;
    }

    *fduplex = 0;

    return OK;

}
