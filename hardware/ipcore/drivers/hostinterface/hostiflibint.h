/**
********************************************************************************
\file   hostiflibint.h

\brief  Host Interface Library - Internal Driver Header

This is the internal driver header.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_HOSTIFLIBINT_H_
#define _INC_HOSTIFLIBINT_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define HOSTIF_MAGIC              0x504C4B00  ///< Host Interface Magic

#define HOSTIF_INSTANCE_COUNT       2   ///< number of supported instances
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Version field in Status/Control - Information

Used to obtain hardware/software mismatch
*/
typedef struct sHostifHwVersion
{
    UINT8           cnt;        ///< Counting field
    tHostifVersion  version;    ///< Version field
} tHostifHwVersion;

/**
\brief Buffer descriptor structure

This structure is used to store the buffer descriptors
*/
typedef struct sHostifBufDesc
{
    UINT32  offset;     ///< Buffer offset within hostif
    UINT    span;       ///< Buffer span [byte]
} tHostifBufDesc;

/**
\brief Buffer map structure

This structure is used to store the buffer base address and size.
*/
typedef struct sHostifBufMap
{
    UINT8*  pBase;  ///< Buffer base address
    UINT    span;   ///< Buffer span [byte]
} tHostifBufMap;

/**
\brief Initialization Parameter structure

This structure is used to forward the initialization from Pcp to host.
*/
typedef struct sHostifInitParam
{
    UINT32          initMemLength; ///< Length of aInitMem
    tHostifBufDesc  aInitMem[HOSTIF_DYNBUF_COUNT + HOSTIF_BUF_COUNT]; ///< Memory map from hostiflib-mem.h
    UINT8           aUser[HOSTIF_USER_INIT_PAR_SIZE]; ///< Space for higher layers
} tHostifInitParam;

/**
\brief Host Interface Instance

Holds the configuration passed to the instance at creation.
*/
typedef struct sHostif
{
    tHostifConfig       config; ///< copy of configuration
    UINT8*              pBase;  ///< base address of host interface
    tHostifIrqCb        apfnIrqCb[kHostifIrqSrcLast]; ///< table that stores the irq callbacks
    tHostifBufMap       aBufMap[kHostifInstIdLast]; ///< Table storing buffer mapping
    tHostifInitParam*   pInitParam; ///< Initialization parameter
    UINT8*              apDynBuf[HOSTIF_DYNBUF_COUNT];
} tHostif;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tHostifReturn hostif_createInt (tHostif* pHostif_p);
tHostifReturn hostif_deleteInt (tHostif* pHostif_p);
tHostifReturn hostif_checkVersion (UINT8* pBase_p, tHostifVersion* pSwVersion_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_HOSTIFLIBINT_H_ */
