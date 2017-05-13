/**
********************************************************************************
\file   obdpi.c

\brief  Process image setup function for CiA302-4

This file contains the implementation of the process image setup functions
for the CiA profile 302-4.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "obdpi.h"

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
#define PI_SUBINDEX_COUNT    252

#ifndef tabentries
#define tabentries(aVar_p)  (sizeof(aVar_p) / sizeof(*(aVar_p)))
#endif


//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing the object linking for the process image

This structure describes the object index range and its data type that shall be
linked into the process image.
*/
typedef struct
{
    UINT            objIndexStart;          ///< Start index of a range
    UINT            objIndexEnd;            ///< End index of a range
    UINT            offsetPI;               ///< Starting offset within the process image
    BOOL            fOutputPI;              ///< Describes whether the objects are linked into an output process image
    tObdSize        entrySize;              ///< Size in Bytes of a single entry in the process image
    UINT            subindexCountPerIndex;  ///< Number of subindexes for each index
} tProcessImageLink;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

tProcessImageLink processImageLink_l[] =
{
//  IndexStart  IndexEnd  OffsetPI OutputPI  EntrySize  SubindexCount
    { 0xA000,   0xA00F,     0,      FALSE,      1,      PI_SUBINDEX_COUNT },
    { 0xA040,   0xA04F,     0,      FALSE,      1,      PI_SUBINDEX_COUNT },
    { 0xA0C0,   0xA0C7,     0,      FALSE,      2,      PI_SUBINDEX_COUNT },
    { 0xA100,   0xA107,     0,      FALSE,      2,      PI_SUBINDEX_COUNT },
    { 0xA1C0,   0xA1C3,     0,      FALSE,      4,      PI_SUBINDEX_COUNT },
    { 0xA200,   0xA203,     0,      FALSE,      4,      PI_SUBINDEX_COUNT },
    { 0xA240,   0xA247,     0,      FALSE,      4,      PI_SUBINDEX_COUNT },
    { 0xA400,   0xA401,     0,      FALSE,      8,      PI_SUBINDEX_COUNT },
    { 0xA440,   0xA441,     0,      FALSE,      8,      PI_SUBINDEX_COUNT },
    { 0xA480,   0xA48F,     0,      TRUE,       1,      PI_SUBINDEX_COUNT },
    { 0xA4C0,   0xA4CF,     0,      TRUE,       1,      PI_SUBINDEX_COUNT },
    { 0xA540,   0xA547,     0,      TRUE,       2,      PI_SUBINDEX_COUNT },
    { 0xA580,   0xA587,     0,      TRUE,       2,      PI_SUBINDEX_COUNT },
    { 0xA640,   0xA643,     0,      TRUE,       4,      PI_SUBINDEX_COUNT },
    { 0xA680,   0xA683,     0,      TRUE,       4,      PI_SUBINDEX_COUNT },
    { 0xA6C0,   0xA6C7,     0,      TRUE,       4,      PI_SUBINDEX_COUNT },
    { 0xA880,   0xA881,     0,      TRUE,       8,      PI_SUBINDEX_COUNT },
    { 0xA8C0,   0xA8C1,     0,      TRUE,       8,      PI_SUBINDEX_COUNT }
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static UINT linkProcessImageRange(UINT objIndexStart_p, UINT objIndexEnd_p,
                                  UINT offsetPI_p, BOOL fOutputPI_p, tObdSize entrySize_p,
                                  UINT subindexCountPerIndex_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Setup process image

The function sets up a process image according to the CiA profile 302-4.

\return The function returns the number of the index that failed to link or 0,
        if no error occurred.
*/
//------------------------------------------------------------------------------
UINT obdpi_setupProcessImage(void)
{
    UINT                        errorIndex = 0;
    size_t                      i;
    tProcessImageLink*          pLink;

    pLink = processImageLink_l;

    for (i = 0; i < tabentries(processImageLink_l); i++, pLink++)
    {
        errorIndex = linkProcessImageRange(pLink->objIndexStart, pLink->objIndexEnd,
                                           pLink->offsetPI, pLink->fOutputPI,
                                           pLink->entrySize, pLink->subindexCountPerIndex);
        if (errorIndex != 0)
            break;
    }

    return errorIndex;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Link process image range

The function links a range of variables to the object dictionary.

\param  objIndexStart_p         Start index of range to link.
\param  objIndexEnd_p           End index of range to link.
\param  offsetPI_p              Offset of range in the process image.
\param  fOutputPI_p             Determines if input image or output image should
                                be used: TRUE = output image, FALSE = imput image
\param  entrySize_p             The size of one process variable.
\param  subindexCountPerIndex_p Number of subindexes per index to be linked.

\return The function returns the number of the index that failed to link or 0,
        if no error occurred.
*/
//------------------------------------------------------------------------------
static tOplkError linkProcessImageRange(UINT objIndexStart_p, UINT objIndexEnd_p,
                                        UINT offsetPI_p, BOOL fOutputPI_p,
                                        tObdSize entrySize_p, UINT subindexCountPerIndex_p)
{
    tOplkError      ret = kErrorOk;
    UINT            errorIndex = 0;
    UINT            varEntries;

    for (; objIndexStart_p <= objIndexEnd_p; objIndexStart_p++,
                                             offsetPI_p += entrySize_p * subindexCountPerIndex_p)
    {
        varEntries = subindexCountPerIndex_p;
        ret = oplk_linkProcessImageObject(objIndexStart_p, 1, offsetPI_p,
                                          fOutputPI_p, entrySize_p, &varEntries);
        if (((ret == kErrorOk) && (varEntries < subindexCountPerIndex_p)) ||
            (ret == kErrorApiPISizeExceeded))
        {
            errorIndex = 0;
            break;
        }

        if (ret != kErrorOk)
        {
            errorIndex = objIndexStart_p;
            break;
        }
    }

    return errorIndex;
}

/// \}
