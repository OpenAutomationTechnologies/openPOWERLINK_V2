/**
********************************************************************************
\file   gpio-c5socarm.c

\brief  GPIOs for Altera Cyclone-V ARM

The file implements the GPIOs on Altera Cyclone-V ARM core used by
openPOWERLINK demo applications.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <alt_generalpurpose_io.h>

#include <oplk/oplk.h>
#include "gpio.h"

#include <system.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

#define LED_OUTPUT_DELAY_US                         500000      // 500ms

#define HPS_LED_ALL_BIT_MASK                        0x0000F000
#define HPS_LED_ALL_TURN_ON                         0x00000000
#define HPS_LED_ALL_TURN_OFF                        0x0000F000
#define HPS_LED_0_TURN_ON                           0x00007000  // GPIO[44] (HPS_LED_0) --> Error Led
#define HPS_LED_1_TURN_ON                           0x0000B000  // GPIO[43] (HPS_LED_1) --> Status Led
#define HPS_LED_2_TURN_ON                           0x0000D000  // GPIO[42] (HPS_LED_2)
#define HPS_LED_3_TURN_ON                           0x0000E000  // GPIO[41] (HPS_LED_3)

#define HPS_LED_0_TURN_OFF                          0x00008000
#define HPS_LED_1_TURN_OFF                          0x00004000
#define HPS_LED_2_TURN_OFF                          0x00002000
#define HPS_LED_3_TURN_OFF                          0x00001000

#define FPGA_LED_ALL_BIT_MASK                       0x0000000F
#define FPGA_LED_ALL_TURN_ON                        0x00000000
#define FPGA_LED_ALL_TURN_OFF                       0x0000000F
#define FPGA_LED_0_TURN_ON                          0x0000000E
#define FPGA_LED_1_TURN_ON                          0x0000000D
#define FPGA_LED_2_TURN_ON                          0x0000000B
#define FPGA_LED_3_TURN_ON                          0x00000007

#define FPGA_LED_0_TURN_OFF                         0x00000001
#define FPGA_LED_1_TURN_OFF                         0x00000002
#define FPGA_LED_2_TURN_OFF                         0x00000004
#define FPGA_LED_3_TURN_OFF                         0x00000008

#define HPS_PB_INT_ALL_BIT_MASK                     0x01E00000  // Interrupt bits for GPIO2

#define HPS_PB_0_ASSERT                             0x01C00000  // GPIO[8] (HPS_PB_0)
#define HPS_PB_1_ASSERT                             0x01A00000  // GPIO[9] (HPS_PB_1)
#define HPS_PB_2_ASSERT                             0x01600000  // GPIO[10](HPS_PB_2)
#define HPS_PB_3_ASSERT                             0x00E00000  // GPIO[11](HPS_PB_3)

#define HPS_PB_ALL_BIT_MASK                         0x01E00000

#define HPS_PB_0_BIT_MASK                           0x01000000  // GPIO[8] (HPS_PB_0)
#define HPS_PB_1_BIT_MASK                           0x00800000  // GPIO[9] (HPS_PB_1)
#define HPS_PB_2_BIT_MASK                           0x00400000  // GPIO[10](HPS_PB_2)
#define HPS_PB_3_BIT_MASK                           0x00200000  // GPIO[11](HPS_PB_3)

#define HPS_DIPSW_ALL_BIT_MASK                      0x001E0000
#define HPS_DIPSW_NET_VAL(portVal)    ((portVal >> 17) & 0xF)

#define HPS_DIPSW_0_BIT_MASK                        0x00100000  // GPIO[4] (HPS_DIPSW_0)
#define HPS_DIPSW_1_BIT_MASK                        0x00080000  // GPIO[5] (HPS_DIPSW_1)
#define HPS_DIPSW_2_BIT_MASK                        0x00040000  // GPIO[6](HPS_DIPSW_2)
#define HPS_DIPSW_3_BIT_MASK                        0x00020000  // GPIO[7](HPS_DIPSW_3)

#define FPGA_PB_ALL_BIT_MASK                        0x00000003

#define FPGA_PB_0_BIT_MASK                          0x00000001
#define FPGA_PB_1_BIT_MASK                          0x00000002

#define FPGA_DIPSW_ALL_BIT_MASK                     0x0000000F

#define FPGA_DIPSW_0_BIT_MASK                       0x00000001
#define FPGA_DIPSW_1_BIT_MASK                       0x00000002
#define FPGA_DIPSW_3_BIT_MASK                       0x00000001
#define FPGA_DIPSW_4_BIT_MASK                       0x00000002

#define FPGA_DIPSW_NET_VAL(portVal)     (portVal & FPGA_DIPSW_ALL_BIT_MASK)

// Determine size of an array
#define ARRAY_COUNT(array)              (sizeof(array) / sizeof(array[0]))


//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

ALT_GPIO_CONFIG_RECORD_t    cfgHpsGPI[] =
{
    { ALT_HLGPI_4, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_0
    { ALT_HLGPI_5, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_1
    { ALT_HLGPI_6, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_2
    { ALT_HLGPI_7, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_DIPSW_3
    { ALT_HLGPI_8, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_PB_0
    { ALT_HLGPI_9, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 },                            // HPS_PB_1
    { ALT_HLGPI_10, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 }, // HPS_PB_2
    { ALT_HLGPI_11, ALT_GPIO_PIN_INPUT, 0, 0, 0, 0 } // HPS_PB_3
};

ALT_GPIO_CONFIG_RECORD_t    cfgHpsGPO[] =
{
    { ALT_GPIO_1BIT_44, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO },  // HPS_LED_0
    { ALT_GPIO_1BIT_43, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO },  // HPS_LED_1
    { ALT_GPIO_1BIT_42, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO },  // HPS_LED_2
    { ALT_GPIO_1BIT_41, ALT_GPIO_PIN_OUTPUT, 0, 0, 0, ALT_GPIO_PIN_DATAZERO }   // HPS_LED_3
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize GPIO module

The function initializes the GPIO module before being used.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_init(void)
{
    ALT_STATUS_CODE    halRet = ALT_E_SUCCESS;

    /* Initialize HPS GPIO */

    // Initialize GPIO module
    halRet = alt_gpio_init();
    if (halRet != ALT_E_SUCCESS)
    {
        goto Exit;
    }

    halRet = alt_gpio_group_config(cfgHpsGPO, ARRAY_COUNT(cfgHpsGPO));  // Setup GPIO LED
    if (halRet != ALT_E_SUCCESS)
    {
        goto Exit;
    }

    halRet = alt_gpio_port_data_write(ALT_GPIO_PORTB, HPS_LED_ALL_BIT_MASK,
                                      HPS_LED_ALL_TURN_OFF);    // clear the Leds
    if (halRet != ALT_E_SUCCESS)
    {
        goto Exit;
    }

    halRet = alt_gpio_group_config(cfgHpsGPI, ARRAY_COUNT(cfgHpsGPI));  // Setup GPIO PUSHBUTTON
    if (halRet != ALT_E_SUCCESS)
    {
        goto Exit;
    }

    halRet = alt_gpio_port_int_disable(ALT_GPIO_PORTC, HPS_PB_INT_ALL_BIT_MASK);    // Enable GPIO interrupts
    if (halRet != ALT_E_SUCCESS)
    {
        goto Exit;
    }

    /* Initialize FPGA GPIO */

    // will be initialized in target intialization

    // clear the Leds
#ifdef LED_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_BASE, FPGA_LED_ALL_TURN_OFF);
#endif

    // Clear the dip switch and push button interrupt status registers
#ifdef HOST_0_BUTTON_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(HOST_0_BUTTON_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(HOST_0_BUTTON_PIO_BASE, FPGA_PB_ALL_BIT_MASK);
#endif

#ifdef HOST_0_DIPSW_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(HOST_0_DIPSW_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(HOST_0_DIPSW_PIO_BASE, FPGA_DIPSW_ALL_BIT_MASK);
#endif

 Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown GPIO module

The function shuts down the GPIO module.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_exit(void)
{
    /* Uninitialize HPS GPIO */
    alt_gpio_uninit();

    /* Uninitialize FPGA GPIO */

    // will be handled by target module
#ifdef HOST_0_BUTTON_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(HOST_0_BUTTON_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(HOST_0_BUTTON_PIO_BASE, FPGA_PB_ALL_BIT_MASK);
#endif
#ifdef HOST_0_DIPSW_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(HOST_0_DIPSW_PIO_BASE, 0x0);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(HOST_0_DIPSW_PIO_BASE, FPGA_DIPSW_ALL_BIT_MASK);
#endif

    // clear the Leds
#ifdef LED_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_BASE, FPGA_LED_ALL_TURN_OFF);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Gets the node switch value

The function returns the node ID set by the node switches.

\return Returns the set node ID

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
UINT8 gpio_getNodeid(void)
{
    UINT8       nodeId;
    UINT32      hpsSwStatus = 0x0;
    UINT32      fpgaSwStatus = 0x0;

    hpsSwStatus = alt_gpio_port_data_read(ALT_GPIO_PORTC, HPS_DIPSW_ALL_BIT_MASK);

#ifdef HOST_0_DIPSW_PIO_BASE
    fpgaSwStatus = IORD_ALTERA_AVALON_PIO_DATA(HOST_0_DIPSW_PIO_BASE);
#endif

    nodeId = (UINT8)((FPGA_DIPSW_NET_VAL(fpgaSwStatus) << HOST_0_DIPSW_PIO_DATA_WIDTH) | HPS_DIPSW_NET_VAL(hpsSwStatus));

    return nodeId;
}

//------------------------------------------------------------------------------
/**
\brief  Gets the application input

The function returns application inputs.

\return Returns the application inputs.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
UINT8 gpio_getAppInput(void)
{
    UINT8    key;

#ifdef HOST_0_BUTTON_PIO_BASE
    key = (UINT8)IORD_ALTERA_AVALON_PIO_EDGE_CAP(HOST_0_BUTTON_PIO_BASE);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(HOST_0_BUTTON_PIO_BASE, FPGA_PB_ALL_BIT_MASK);
#else
    key = 0;
#endif

    return key;
}

//------------------------------------------------------------------------------
/**
\brief  Sets the application output

The function sets the application outputs.

\param  val_p               Determines the value to be set to the output

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_setAppOutputs(UINT32 val_p)
{
#ifdef LED_PIO_BASE
    IOWR_ALTERA_AVALON_PIO_DATA(LED_PIO_BASE, ~val_p);
#endif
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}
