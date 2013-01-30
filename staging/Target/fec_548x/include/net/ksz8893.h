
//#include   "timer.h"

#define   FEC_MII_SPEED              (((unsigned int)( MCF_BUSCLK/ 5000000) + 1) << 1)

#define   KSZ8893_CHIPID0            0x00    // chip family (0x88)
#define   KSZ8893_CHIPID1            0x01    // revision, start switch
#define   KSZ8893_GLOBALCTRL4        0x06    // repeater mode, port 3 configuration
#define   KSZ8893_PORT1CTRL12        0x1C    // port 1 configuration (auto negotiation, speed)
#define   KSZ8893_PORT2CTRL12        0x2C    // port 2 configuration (auto negotiation, speed)

#define   KSZ8893_CHIPID0_VAL        0x88    // chip family (0x88)
#define   KSZ8893_CHIPID1_START      0x01    // start switch
#define   KSZ8893_GLOBALCTRL4_REP    0x80    // repeater mode
#define   KSZ8893_GLOBALCTRL4_HALF   0x40    // port 3: half duplex
#define   KSZ8893_PORTCTRL12_100     0x40    // port configuration: 100 MBit

#define   KSZ8893_READ(reg)          ((reg << 18) | 0x48020000)  // read specified register
#define   KSZ8893_WRITE(reg, data)   ((reg << 18) | 0x40020000 | data)  // write data to register
#define   KSZ8893_DATA(val)          (val & 0x000000FF)          // returns data portion of value


int ksz8893_init_transceiver(unsigned long base_addr, int *fduplex);
