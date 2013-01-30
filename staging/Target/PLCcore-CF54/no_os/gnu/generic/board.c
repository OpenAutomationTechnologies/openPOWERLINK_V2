#include "arch.h"
#include "mcfuart.h"

char    ident[] = "PLCcore-MCF5484";
char    copyright[] = "(c) 2006 SYS TEC electronic GmbH";

extern unsigned long board_type_id;        // 2006/06/04 -rs: board type identification (evaluated from Linux, not used by CoLilo itself)
extern unsigned int downloadPort;
extern unsigned int downloadBaud;          // 2006/06/08 -rs
extern unsigned int image_size;
extern unsigned int decompress_image_size;
extern unsigned char *xfer_addr;
extern unsigned char *down_addr;
extern unsigned char *dest_addr;
extern unsigned char *source_addr;
extern unsigned char* flash_addr;           // 2005/09/22  -rs


extern unsigned long consoleBase;

extern unsigned int fec_number;
extern char board_mac_address[];
extern char board_ip_address[];
extern char board_gateway[];
extern char board_broadcast[];              // 2006/06/04 -rs: (evaluated from Linux, not used by CoLilo itself)
extern char board_netmask[];
extern char board_tftp_image_name[];
extern char board_tftp_server_ip[];
extern char board_command_line[];
extern int  config_kernel_flash;
extern int  boot_autostart;

/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#define DEFAULT_IP_ADDRESS "192.168.1.89"
#define DEFAULT_NETMASK "255.255.0.0"
#define DEFAULT_MAC_ADDRESS "00:01:02:03:AA:BB"
#define DEFAULT_GATEWAY "192.168.10.1"
#define DEFAULT_IMAGE "linux_tftp.bin"
#define DEFAULT_TFTP_SERVER "192.168.1.88"
#define DEFAULT_COMMAND_LINE "root=/dev/ram"
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
#define DEFAULT_BOARD_TYPE_ID 0             // 2006/06/04 -rs (DefaultID = unknown board)
#define DEFAULT_SER_PORT 0
#define DEFAULT_SER_BAUD 19200
#define DEFAULT_IP_ADDRESS "192.168.10.248"
#define DEFAULT_NETMASK "255.255.255.0"
#define DEFAULT_BROADCAST "192.168.10.255"  // 2006/06/04
#define DEFAULT_MAC_ADDRESS "00:CF:54:85:CF:01"
#define DEFAULT_GATEWAY "0.0.0.0"
#define DEFAULT_IMAGE "image.bin"
#define DEFAULT_TFTP_SERVER "192.168.10.11"
#define DEFAULT_COMMAND_LINE "root=/dev/ram"
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/


void configureConsole()
{
    consoleBase = MCFUART_BASE1;
    //---------------------------------------------------
    // 2006/06/08 -rs
    //---------------------------------------------------
     configureSerial(consoleBase, 19200, MCF_CLK);
    //---------------------------------------------------
//    configureSerial (consoleBase, downloadBaud, MCF_CLK);
    //---------------------------------------------------
}

void configureAuxSerial()
{
    configureSerial(MCFUART_BASE2, 115200, MCF_CLK);
}

void setLED() {};

void setImageParams()
{
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    downloadPort    = 1;
    image_size  = 0x00130000;
    decompress_image_size = image_size;
    source_addr = (unsigned char *)0x7fc10000;
    down_addr   = (unsigned char *)0x00001000;
    xfer_addr   = (unsigned char *)0x01000;
    dest_addr   = (unsigned char *)0x01000;
    fec_number = 0;
    config_kernel_flash = 0;
    boot_autostart = 0;

    strcpy(board_ip_address, DEFAULT_IP_ADDRESS);
    strcpy(board_netmask, DEFAULT_NETMASK);
    strcpy(board_mac_address, DEFAULT_MAC_ADDRESS);
    strcpy(board_gateway, DEFAULT_GATEWAY);
    strcpy(board_tftp_image_name, DEFAULT_IMAGE);
    strcpy(board_tftp_server_ip, DEFAULT_TFTP_SERVER);
    strcpy(board_command_line, DEFAULT_COMMAND_LINE);
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
    board_type_id         = DEFAULT_BOARD_TYPE_ID;          // 2006/06/04 -rs
    downloadPort          = DEFAULT_SER_PORT;               // 2006/06/08 -rs  serial interface port number
    downloadBaud          = DEFAULT_SER_BAUD;               // 2006/06/08 -rs
    image_size            = 0x00130000;                     // ??? (correct size is set by TFTP download routine)
    decompress_image_size = image_size;
    source_addr           = (unsigned char *)0x7fc10000;    // ???
    down_addr             = (unsigned char *)0x00001000;    // Destination address for TFTP-Download of Linux-Image in RAM
    xfer_addr             = (unsigned char *)0x00002000;    // Linux entry point for boot
    dest_addr             = (unsigned char *)0x00001000;    // Start of decompressed Linux in RAM
    flash_addr            = (unsigned char *)0xff000000;    // Start of Linux-Image in Flash
    fec_number            = 0;
    config_kernel_flash   = 0;
    boot_autostart        = 0;

    strcpy(board_mac_address, DEFAULT_MAC_ADDRESS);
    strcpy(board_ip_address, DEFAULT_IP_ADDRESS);
    strcpy(board_gateway, DEFAULT_GATEWAY);
    strcpy(board_broadcast, DEFAULT_BROADCAST);             // 2006/06/04 -rs: (evaluated from Linux, not used by CoLilo itself)
    strcpy(board_netmask, DEFAULT_NETMASK);
    strcpy(board_tftp_image_name, DEFAULT_IMAGE);
    strcpy(board_tftp_server_ip, DEFAULT_TFTP_SERVER);
    strcpy(board_command_line, DEFAULT_COMMAND_LINE);
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
}

void setupBoard()
{
    /* Flash */
/*
    MCF_FBCS_CSAR0 = MCF_FBCS_CSAR_BA(FLASH_ADDRESS);
    MCF_FBCS_CSCR0 = (MCF_FBCS_CSCR_PS_32
                        | MCF_FBCS_CSCR_AA
                        | MCF_FBCS_CSCR_WS(6));
    MCF_FBCS_CSMR0 = (MCF_FBCS_CSMR_BAM_32M
                        | MCF_FBCS_CSMR_V);
*/
    /* Enable the output lines for the serial ports */
    MCF_GPIO_PAR_PSC0 = (0
        | MCF_GPIO_PAR_PSC0_PAR_TXD0
        | MCF_GPIO_PAR_PSC0_PAR_RXD0);
    MCF_GPIO_PAR_PSC1 = (0
        | MCF_GPIO_PAR_PSC1_PAR_TXD1
        | MCF_GPIO_PAR_PSC1_PAR_RXD1);
    MCF_FBCS_CSAR1 = 0xF0000000;


    /* Enable DIP-Switch2 as autorun button                  */
    /* (configure pin DACK1 (PDMA3) for general purpose I/O) */
    MCF_GPIO_PAR_DMA &= 0x3F;               // clr b6+b7

}

void setupDRAM (void)
{
    /* Initialize DDR DIMMs on the Verdi board */

    /*
     * Check to see if the SDRAM has already been initialized
     * by a run control tool
     */
    if (!(MCF_SDRAMC_SDCR & MCF_SDRAMC_SDCR_REF))
    {

        /* Basic configuration and initialization */
        MCF_SDRAMC_SDRAMDS = (0
            | MCF_SDRAMC_SDRAMDS_SB_E(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
            | MCF_SDRAMC_SDRAMDS_SB_C(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
            | MCF_SDRAMC_SDRAMDS_SB_A(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
            | MCF_SDRAMC_SDRAMDS_SB_S(MCF_SDRAMC_SDRAMDS_DRIVE_8MA)
            | MCF_SDRAMC_SDRAMDS_SB_D(MCF_SDRAMC_SDRAMDS_DRIVE_8MA) );

        MCF_SDRAMC_CS0CFG = (0
            | MCF_SDRAMC_CSnCFG_CSBA(SDRAM_ADDRESS)
            | MCF_SDRAMC_CSnCFG_CSSZ(MCF_SDRAMC_CSnCFG_CSSZ_32MBYTE) );


        MCF_SDRAMC_SDCFG1 = (0
            | MCF_SDRAMC_SDCFG1_SRD2RW(5)
            | MCF_SDRAMC_SDCFG1_SWT2RD(3)
            | MCF_SDRAMC_SDCFG1_RDLAT(7)
            | MCF_SDRAMC_SDCFG1_ACT2RW(2)
            | MCF_SDRAMC_SDCFG1_PRE2ACT(2)
            | MCF_SDRAMC_SDCFG1_SRD2RW(5)
            | MCF_SDRAMC_SDCFG1_REF2ACT(9)
            | MCF_SDRAMC_SDCFG1_WTLAT(3)   );

        MCF_SDRAMC_SDCFG2 = (0
            | MCF_SDRAMC_SDCFG2_BRD2PRE(2)
            | MCF_SDRAMC_SDCFG2_BWT2RW(4)
            | MCF_SDRAMC_SDCFG2_BRD2WT(3)
            | MCF_SDRAMC_SDCFG2_BL(3)      );

        /* Precharge and enable write to SDMR */
        MCF_SDRAMC_SDCR = (0
            | MCF_SDRAMC_SDCR_MODE_EN
            | MCF_SDRAMC_SDCR_CKE
            | MCF_SDRAMC_SDCR_DDR
            | MCF_SDRAMC_SDCR_MUX(0)
            | MCF_SDRAMC_SDCR_RCNT(0xF)
            | MCF_SDRAMC_SDCR_IPALL      );

        /* Write extended mode register */
        MCF_SDRAMC_SDMR = (0
            | MCF_SDRAMC_SDMR_BNKAD_LEMR
            | MCF_SDRAMC_SDMR_AD(0x0)
            | MCF_SDRAMC_SDMR_CMD        );

        /* Write mode register and reset DLL */
        MCF_SDRAMC_SDMR = (0
            | MCF_SDRAMC_SDMR_BNKAD_LMR
            | MCF_SDRAMC_SDMR_AD(0x588)
            | MCF_SDRAMC_SDMR_CMD        );

        /* Execute a PALL command */
        MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR
            | MCF_SDRAMC_SDCR_IPALL      );

        /* Perform two REF cycles */
        MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR       /*perform first refresh*/
            | MCF_SDRAMC_SDCR_IREF       );

        MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR       /*perform second refresh*/
            | MCF_SDRAMC_SDCR_IREF       );

        /* Write mode register and clear reset DLL */
        MCF_SDRAMC_SDMR = (0
            | MCF_SDRAMC_SDMR_BNKAD_LMR
            | MCF_SDRAMC_SDMR_AD(0x188)
            | MCF_SDRAMC_SDMR_CMD        );

        /* Enable auto refresh and lock SDMR */
        MCF_SDRAMC_SDCR = (MCF_SDRAMC_SDCR
            & ~MCF_SDRAMC_SDCR_MODE_EN
            | MCF_SDRAMC_SDCR_REF
            | MCF_SDRAMC_SDCR_DQS_OE(0xF) );
    }
}


//---------------------------------------------------------------------------
// 2006/01/30 -rs
//---------------------------------------------------------------------------
int GetAutoRunState (void)
{

unsigned char  bPort;
int            fAutoRunState;


    /* get state of DIP-Switch2 alias autorun button (pin DACK1 (PDMA3)) */
    bPort = MCF_GPIO_PPDSDR_DMA;
    if ( (bPort & 0x08) )
    {
        fAutoRunState = 0;
    }
    else
    {
        fAutoRunState = 1;
    }


    return (fAutoRunState);

}

//---------------------------------------------------------------------------

