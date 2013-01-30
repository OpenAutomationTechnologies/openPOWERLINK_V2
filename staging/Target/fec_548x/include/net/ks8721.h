
//#include   "timer.h"

#define   FEC_MII_SPEED				 (((unsigned int)( MCF_BUSCLK/ 5000000) + 1) << 1)

#define   KS8721_CTRL				 0x00
#define   KS8721_ANADV				 0x04
#define   KS8721_STAT				 0x01

#define   KS8721_CTRL_RESET			 0x8000
#define   KS8721_ANADV_ADV_ALL		 0x01E1
#define   KS8721_CTRL_AN_ENABLE		 0x1280
//#define   KS8721_CTRL_DEFAULT_MODE   0x2100
#define   KS8721_CTRL_DEFAULT_MODE_10    0x0000    // half duplex , 10 MBit
#define   KS8721_CTRL_DEFAULT_MODE_100   0x2000    // half duplex , 100 MBit
#define   KS8721_STAT_ANCOMPLETE	 0x0020
#define   KS8721_STAT_LINK			 0x0004


#define   FEC_DEFAULT_ADDR   		 1
#define   KS8721_STAT_FDUPLEX		 0x5000
#define   KS8721_AUTONEG_TIMEOUT	 5000/TIMER_PERIOD


int ks8721_init_transceiver(unsigned long base_addr, int *fduplex);
