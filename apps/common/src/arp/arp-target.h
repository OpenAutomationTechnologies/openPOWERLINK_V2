/**
********************************************************************************
\file   arp-target.h

\brief  ARP demo target header

This file contains the target specific definitions like endian conversion.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_arp_target_H_
#define _INC_arp_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _LINUX_)
#include <arpa/inet.h>
#else
#if CHECK_IF_BIG_ENDIAN()
// Big endian => no swap needed
#define htons(x)        (x)
#define htonl(x)        (x)
#define ntohs(x)        (x)
#define ntohl(x)        (x)
#else
// Little endian => swap needed
// Swap long: 0x00C0FFEE --> 0xEEFFC000
#define ARP_SWAPL(x)    ((((x) >> 24) & 0x000000FF) | \
                         (((x) >> 8) & 0x0000FF00) |  \
                         (((x) & 0x000000FF) << 24) | \
                         (((x) & 0x0000FF00) << 8))

// Swap short: 0xC0FE --> 0xFEC0
#define ARP_SWAPS(x)    ((((x) >> 8) & 0x00FF) | \
                         (((x) << 8) & 0xFF00))

#define htons(x)        ARP_SWAPS(x)
#define htonl(x)        ARP_SWAPL(x)
#define ntohs(x)        ARP_SWAPS(x)
#define ntohl(x)        ARP_SWAPL(x)
#endif //CHECK_IF_BIG_ENDIAN
#endif //DEV_SYSTEM

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_arp_target_H_ */
