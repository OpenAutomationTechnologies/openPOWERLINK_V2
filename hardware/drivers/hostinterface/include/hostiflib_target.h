/**
********************************************************************************
\file   hostiflib_target.h

\brief  Host Interface Library - Target header file

This header file defines target specific macros (e.g. data types) and selects
the target specific header file (e.g. hostiflib_nios.h).

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2014, Kalycito Infotech Private Limited
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

#ifndef _INC_hostiflib_target_H_
#define _INC_hostiflib_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>

#if defined(__NIOS2__)

#include "hostiflib_nios.h"

#elif defined(__MICROBLAZE__)

#include "hostiflib_microblaze.h"

#else

#error "Target is not supported! Please revise hostiflib_target.h"

#endif

#ifndef HOSTIF_INLINE
#define HOSTIF_INLINE
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

/**
\name Data types
If the following data types are not defined in the environment, then they are
set to those provided by stdint.h.
*/
/**@{*/
#ifndef UINT8
#define UINT8       uint8_t
#endif

#ifndef UINT16
#define UINT16      uint16_t
#endif

#ifndef UINT32
#define UINT32      uint32_t
#endif

#ifndef UINT
#define UINT        unsigned int
#endif

#ifndef BOOL
#define BOOL        uint8_t
#endif

#ifndef FALSE
#define FALSE       0x00
#endif

#ifndef TRUE
#define TRUE        0xFF
#endif
/**@}*/

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_hostiflib_target_H_ */
