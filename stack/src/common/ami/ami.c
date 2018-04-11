/**
********************************************************************************
\file   ami/ami.c

\brief  Generic implementation of the Abstract Memory Interface (ami)

This file implements the AMI interface for architectures where access to
unaligned addresses is not possible. (This implementation always copies
bytewise)

\ingroup module_ami
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2018, embedded brains GmbH
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/ami.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Set Uint16 to big endian

Sets a 16 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint16Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint16Be(void* pAddr_p, UINT16 uint16Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint16Val_p >> 8);
    pAddr[1] = (UINT8)(uint16Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint16 to little endian

Sets a 16 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint16Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint16Le(void* pAddr_p, UINT16 uint16Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint16Val_p >> 0);
    pAddr[1] = (UINT8)(uint16Val_p >> 8);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint16 from big endian

Reads a 16 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT16
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT16 ami_getUint16Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT16)pAddr[0] << 8) | ((UINT16)pAddr[1] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint16 from little endian

Reads a 16 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT16
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT16 ami_getUint16Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT16)pAddr[0] << 0) | ((UINT16)pAddr[1] << 8);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint24 to big endian

Sets a 24 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint32Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint24Be(void* pAddr_p, UINT32 uint32Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint32Val_p >> 16);
    pAddr[1] = (UINT8)(uint32Val_p >> 8);
    pAddr[2] = (UINT8)(uint32Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint24 to little endian

Sets a 24 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint32Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint24Le(void* pAddr_p, UINT32 uint32Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint32Val_p >> 0);
    pAddr[1] = (UINT8)(uint32Val_p >> 8);
    pAddr[2] = (UINT8)(uint32Val_p >> 16);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint24 from big endian

Reads a 24 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT32
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT32 ami_getUint24Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT32)pAddr[0] << 16) | ((UINT32)pAddr[1] << 8) |
           ((UINT32)pAddr[2] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint24 from little endian

Reads a 24 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT32
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT32 ami_getUint24Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT32)pAddr[0] << 0) | ((UINT32)pAddr[1] << 8) |
           ((UINT32)pAddr[2] << 16);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint32 to big endian

Sets a 32 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint32Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint32Be(void* pAddr_p, UINT32 uint32Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint32Val_p >> 24);
    pAddr[1] = (UINT8)(uint32Val_p >> 16);
    pAddr[2] = (UINT8)(uint32Val_p >> 8);
    pAddr[3] = (UINT8)(uint32Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint32 to little endian

Sets a 32 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint32Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint32Le(void* pAddr_p, UINT32 uint32Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint32Val_p >> 0);
    pAddr[1] = (UINT8)(uint32Val_p >> 8);
    pAddr[2] = (UINT8)(uint32Val_p >> 16);
    pAddr[3] = (UINT8)(uint32Val_p >> 24);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint32 from big endian

Reads a 32 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT32
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT32 ami_getUint32Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT32)pAddr[0] << 24) | ((UINT32)pAddr[1] << 16) |
           ((UINT32)pAddr[2] << 8) | ((UINT32)pAddr[3] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint32 from little endian

Reads a 32 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT32
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT32 ami_getUint32Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT32)pAddr[0] << 0) | ((UINT32)pAddr[1] << 8) |
           ((UINT32)pAddr[2] << 16) | ((UINT32)pAddr[3] << 24);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint40 to big endian

Sets a 40 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint40Be(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 32);
    pAddr[1] = (UINT8)(uint64Val_p >> 24);
    pAddr[2] = (UINT8)(uint64Val_p >> 16);
    pAddr[3] = (UINT8)(uint64Val_p >> 8);
    pAddr[4] = (UINT8)(uint64Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint40 to little endian

Sets a 40 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint40Le(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 0);
    pAddr[1] = (UINT8)(uint64Val_p >> 8);
    pAddr[2] = (UINT8)(uint64Val_p >> 16);
    pAddr[3] = (UINT8)(uint64Val_p >> 24);
    pAddr[4] = (UINT8)(uint64Val_p >> 32);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint40 from big endian

Reads a 40 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint40Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 32) | ((UINT64)pAddr[1] << 24) |
           ((UINT64)pAddr[2] << 16) | ((UINT64)pAddr[3] << 8) |
           ((UINT64)pAddr[4] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint40 from little endian

Reads a 40 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint40Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 0) | ((UINT64)pAddr[1] << 8) |
           ((UINT64)pAddr[2] << 16) | ((UINT64)pAddr[3] << 24) |
           ((UINT64)pAddr[4] << 32);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint48 to big endian

Sets a 48 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint48Be(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 40);
    pAddr[1] = (UINT8)(uint64Val_p >> 32);
    pAddr[2] = (UINT8)(uint64Val_p >> 24);
    pAddr[3] = (UINT8)(uint64Val_p >> 16);
    pAddr[4] = (UINT8)(uint64Val_p >> 8);
    pAddr[5] = (UINT8)(uint64Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint48 to little endian

Sets a 48 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint48Le(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 0);
    pAddr[1] = (UINT8)(uint64Val_p >> 8);
    pAddr[2] = (UINT8)(uint64Val_p >> 16);
    pAddr[3] = (UINT8)(uint64Val_p >> 24);
    pAddr[4] = (UINT8)(uint64Val_p >> 32);
    pAddr[5] = (UINT8)(uint64Val_p >> 40);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint48 from big endian

Reads a 48 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint48Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 40) | ((UINT64)pAddr[1] << 32) |
           ((UINT64)pAddr[2] << 24) | ((UINT64)pAddr[3] << 16) |
           ((UINT64)pAddr[4] << 8) | ((UINT64)pAddr[5] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint48 from little endian

Reads a 48 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint48Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 0) | ((UINT64)pAddr[1] << 8) |
           ((UINT64)pAddr[2] << 16) | ((UINT64)pAddr[3] << 24) |
           ((UINT64)pAddr[4] << 32) | ((UINT64)pAddr[5] << 40);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint56 to big endian

Sets a 56 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint56Be(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 48);
    pAddr[1] = (UINT8)(uint64Val_p >> 40);
    pAddr[2] = (UINT8)(uint64Val_p >> 32);
    pAddr[3] = (UINT8)(uint64Val_p >> 24);
    pAddr[4] = (UINT8)(uint64Val_p >> 16);
    pAddr[5] = (UINT8)(uint64Val_p >> 8);
    pAddr[6] = (UINT8)(uint64Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint56 to little endian

Sets a 56 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint56Le(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 0);
    pAddr[1] = (UINT8)(uint64Val_p >> 8);
    pAddr[2] = (UINT8)(uint64Val_p >> 16);
    pAddr[3] = (UINT8)(uint64Val_p >> 24);
    pAddr[4] = (UINT8)(uint64Val_p >> 32);
    pAddr[5] = (UINT8)(uint64Val_p >> 40);
    pAddr[6] = (UINT8)(uint64Val_p >> 48);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint56 from big endian

Reads a 56 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint56Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 48) | ((UINT64)pAddr[1] << 40) |
           ((UINT64)pAddr[2] << 32) | ((UINT64)pAddr[3] << 24) |
           ((UINT64)pAddr[4] << 16) | ((UINT64)pAddr[5] << 8) |
           ((UINT64)pAddr[6] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint56 from little endian

Reads a 56 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint56Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 0) | ((UINT64)pAddr[1] << 8) |
           ((UINT64)pAddr[2] << 16) | ((UINT64)pAddr[3] << 24) |
           ((UINT64)pAddr[4] << 32) | ((UINT64)pAddr[5] << 40) |
           ((UINT64)pAddr[6] << 48);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint64 to big endian

Sets a 64 bit value to a buffer in big endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint64Be(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 56);
    pAddr[1] = (UINT8)(uint64Val_p >> 48);
    pAddr[2] = (UINT8)(uint64Val_p >> 40);
    pAddr[3] = (UINT8)(uint64Val_p >> 32);
    pAddr[4] = (UINT8)(uint64Val_p >> 24);
    pAddr[5] = (UINT8)(uint64Val_p >> 16);
    pAddr[6] = (UINT8)(uint64Val_p >> 8);
    pAddr[7] = (UINT8)(uint64Val_p >> 0);
}

//------------------------------------------------------------------------------
/**
\brief    Set Uint64 to little endian

Sets a 64 bit value to a buffer in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      uint64Val_p         The source value to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setUint64Le(void* pAddr_p, UINT64 uint64Val_p)
{
    UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    pAddr[0] = (UINT8)(uint64Val_p >> 0);
    pAddr[1] = (UINT8)(uint64Val_p >> 8);
    pAddr[2] = (UINT8)(uint64Val_p >> 16);
    pAddr[3] = (UINT8)(uint64Val_p >> 24);
    pAddr[4] = (UINT8)(uint64Val_p >> 32);
    pAddr[5] = (UINT8)(uint64Val_p >> 40);
    pAddr[6] = (UINT8)(uint64Val_p >> 48);
    pAddr[7] = (UINT8)(uint64Val_p >> 56);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint64 from big endian

Reads a 64 bit value from a buffer in big endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint64Be(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 56) | ((UINT64)pAddr[1] << 48) |
           ((UINT64)pAddr[2] << 40) | ((UINT64)pAddr[3] << 32) |
           ((UINT64)pAddr[4] << 24) | ((UINT64)pAddr[5] << 16) |
           ((UINT64)pAddr[6] << 8) | ((UINT64)pAddr[7] << 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get Uint64 from little endian

Reads a 64 bit value from a buffer in little endian

\param[in]      pAddr_p             Pointer to the source buffer

\return UINT64
\retval Value       The data in platform endian

\ingroup module_ami
*/
//------------------------------------------------------------------------------
UINT64 ami_getUint64Le(const void* pAddr_p)
{
    const UINT8* pAddr = pAddr_p;

    // Check parameter validity
    ASSERT(pAddr != NULL);

    return ((UINT64)pAddr[0] << 0) | ((UINT64)pAddr[1] << 8) |
           ((UINT64)pAddr[2] << 16) | ((UINT64)pAddr[3] << 24) |
           ((UINT64)pAddr[4] << 32) | ((UINT64)pAddr[5] << 40) |
           ((UINT64)pAddr[6] << 48) | ((UINT64)pAddr[7] << 56);
}

//------------------------------------------------------------------------------
/**
\brief    Set time of day

Sets the time of day (CANopen timestamp) to memory in little endian

\param[out]     pAddr_p             Pointer to the destination buffer
\param[in]      pTimeOfDay_p        Pointer to the source memory to convert

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_setTimeOfDay(void* pAddr_p, const tTimeOfDay* pTimeOfDay_p)
{
    // Check parameter validity
    ASSERT(pAddr_p != NULL);
    ASSERT(pTimeOfDay_p != NULL);

    ami_setUint32Le(((UINT8*)pAddr_p), pTimeOfDay_p->msec & 0x0FFFFFFF);
    ami_setUint16Le(((UINT8*)pAddr_p) + 4, pTimeOfDay_p->days);
}

//------------------------------------------------------------------------------
/**
\brief    Get time of day

Get the time of day (CANopen timestamp) from memory in little endian

\param[in]      pAddr_p             Pointer to the source memory to convert
\param[out]     pTimeOfDay_p        Pointer to the destination buffer

\ingroup module_ami
*/
//------------------------------------------------------------------------------
void ami_getTimeOfDay(const void* pAddr_p, tTimeOfDay* pTimeOfDay_p)
{
    // Check parameter validity
    ASSERT(pAddr_p != NULL);
    ASSERT(pTimeOfDay_p != NULL);

    pTimeOfDay_p->msec = ami_getUint32Le(((const UINT8*)pAddr_p)) & 0x0FFFFFFF;
    pTimeOfDay_p->days = ami_getUint16Le(((const UINT8*)pAddr_p) + 4);
}
