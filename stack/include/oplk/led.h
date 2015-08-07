/**
********************************************************************************
\file   oplk/led.h

\brief  Definitions for user LED module

This file contains definitions for the user LED module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_oplk_led_H_
#define _INC_oplk_led_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define LED_DURATION_FLICKERING    50      // [ms]
#define LED_DURATION_BLINKING      200     // [ms]
#define LED_DURATION_FLASH_ON      200     // [ms]
#define LED_DURATION_FLASH_OFF     1000    // [ms]

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
    kLedModeInit           = 0x00,  ///< Led Initilization.
    kLedModeOff            = 0x01,  ///< Led off.
    kLedModeOn             = 0x02,  ///< Led on.
    kLedModeFlickering     = 0x03,  ///< Led On for 50ms and Off for 50ms.
    kLedModeBlinking       = 0x04,  ///< Led On for 200ms and Off for 200ms.
    kLedModeSingleFlash    = 0x05,  ///< Led On for 200ms and then it switch Off.
    kLedModeDoubleFlash    = 0x06,  ///< Led On for 200ms and Off for 1000ms, repeat twice.
    kLedModeTripleFlash    = 0x07,  ///< Led On for 200ms and Off for 1000ms, repeat thrice.
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

#endif /* _INC_oplk_led_H_ */
