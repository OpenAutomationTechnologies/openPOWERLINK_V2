/**
********************************************************************************
\file   common/led.h

\brief  Common definitions for LED module

This file contains common definitions for the LED module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_common_led_H_
#define _INC_common_led_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
 * \brief   Enumeration for valid LED modes
 *
 * The enumeration lists all valid LED modes.
 */
typedef enum
{
    kLedModeInit           = 0x00,  ///< LED initialization.
    kLedModeOff            = 0x01,  ///< LED off.
    kLedModeOn             = 0x02,  ///< LED on.
    kLedModeFlickering     = 0x03,  ///< LED on for 50ms and off for 50ms.
    kLedModeBlinking       = 0x04,  ///< LED on for 200ms and off for 200ms.
    kLedModeSingleFlash    = 0x05,  ///< LED on for 200ms and then off for 1000ms.
    kLedModeDoubleFlash    = 0x06,  ///< Blink twice and then off for 1000ms.
    kLedModeTripleFlash    = 0x07,  ///< Blink thrice and then off for 1000ms.
} eLedMode;

/**
\brief LED mode data type

Data type for the enumerator \ref eLedMode.
*/
typedef UINT32 tLedMode;

/**
 * \brief   Valid LED types
 *
 * The structure defines all valid LED types used by POWERLINK.
 */
typedef enum
{
    kLedTypeStatus   = 0x00,    ///< POWERLINK Status LED
    kLedTypeError    = 0x01,    ///< POWERLINK Error LED
} eLedType;

/**
\brief LED type data type

Data type for the enumerator \ref eLedType.
*/
typedef UINT32 tLedType;

#endif /* _INC_common_led_H_ */
