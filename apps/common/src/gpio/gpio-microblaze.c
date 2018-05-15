/**
********************************************************************************
\file   gpio-microblaze.c

\brief  GPIOs for Xilinx microblaze

The file implements the GPIOs on Xilinx microblaze used by openPOWERLINK demo
applications.

\ingroup module_app_common
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
#include <xparameters.h>
#include <xgpio_l.h>
#include <oplk/oplk.h>
#include "gpio.h"

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
#ifdef XPAR_NODE_SWITCHES_BASEADDR
#define NODE_SWITCH_BASE    XPAR_NODE_SWITCHES_BASEADDR
#endif // XPAR_NODE_SWITCHES_BASEADDR

#ifdef XPAR_GPIO_INPUTS_BASEADDR
#define GPIO_INPUTS_BASE    XPAR_GPIO_INPUTS_BASEADDR
#endif // XPAR_GPIO_INPUTS_BASEADDR

#ifdef XPAR_GPIO_OUTPUTS_BASEADDR
#define GPIO_OUTPUTS_BASE   XPAR_GPIO_OUTPUTS_BASEADDR
#endif // XPAR_GPIO_OUTPUTS_BASEADDR

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize GPIO module

The function initializes the GPIO module.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_init(void)
{

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
    UINT8   nodeid;

#if defined(NODE_SWITCH_BASE)
    nodeid = XGpio_ReadReg(NODE_SWITCH_BASE, 0);
#else
    nodeid = 0;
#endif

    return nodeid;
}

//------------------------------------------------------------------------------
/**
\brief  Gets the application input

The function returns application inputs.

\return Returns the application inputs.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
UINT32 gpio_getAppInput(void)
{
    UINT32  input;

#if defined(GPIO_INPUTS_BASE)
    input = XGpio_ReadReg(GPIO_INPUTS_BASE, 0);
#else
    input = 0;
#endif

    return input;
}

//------------------------------------------------------------------------------
/**
\brief  Sets the application output

The function sets the application outputs.

\param[in]      val_p               Determines the value to be set to the output

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void gpio_setAppOutputs(UINT32 val_p)
{
#if defined(GPIO_OUTPUTS_BASE)
    XGpio_WriteReg(GPIO_OUTPUTS_BASE, XGPIO_DATA_OFFSET, val_p);
#else
    UNUSED_PARAMETER(val_p);
#endif
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
