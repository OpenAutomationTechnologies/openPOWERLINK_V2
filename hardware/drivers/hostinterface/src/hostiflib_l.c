/**
********************************************************************************
\file   hostiflib_l.c

\brief  Host Interface Library - Low Level Driver Source

The Host Interface Low Level Driver provides access to the status control
register structures.

\ingroup module_hostiflib
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib.h"
#include "hostiflib_l.h"
#include "hostiflib_target.h"


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

/**
\name Status/Control Sub-Register Offsets
The offsets of the sub-registers of the status control register
*/
/**@{*/
#define HOSTIF_SC_INFO_OFFS             0x0000U ///< Information
#define HOSTIF_SC_RES0_OFFS             0x0100U ///< reserved
#define HOSTIF_SC_CONT_OFFS             0x0200U ///< Control
#define HOSTIF_SC_SYNC_OFFS             0x0300U ///< Synchronization
#define HOSTIF_SC_DYNB_OFFS             0x0400U ///< Dynamic buffer
#define HOSTIF_SC_RES1_OFFS             0x0500U ///< reserved
#define HOSTIF_SC_RES2_OFFS             0x0600U ///< reserved
#define HOSTIF_SC_RES3_OFFS             0x0700U ///< reserved
#define HOSTIF_SC_HIGH_ADDR             0x07FFU ///< high address
/**@}*/

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Status/Control - Information

The information sub-registers enable to identify a correctly working hardware.
*/
typedef struct sScInfo
{
    volatile UINT32     magic;      ///< Magic Word (="PLK\0")
    volatile UINT32     version;    ///< Version fields
    volatile UINT32     bootBase;   ///< Boot base address
    volatile UINT32     initBase;   ///< Init base address
} tScInfo;

/**
\brief Status/Control - Control

The control sub-registers provide basic Pcp-to-Host communication features.
*/
typedef struct sScCont
{
    volatile UINT16     bridgeEnable; ///< enable the bridge logic
    volatile UINT16     RESERVED0;    ///< reserved
    volatile UINT16     command;      ///< command word
    volatile UINT16     state;        ///< state word
    volatile UINT16     ret;          ///< return word
    volatile UINT16     heartbeat;    ///< heart beat word
    volatile UINT8      RESERVED1;    ///< reserved
    volatile UINT8      RESERVED2;    ///< reserved
    volatile UINT16     RESERVED3;    ///< reserved
    volatile UINT16     RESERVED4;    ///< reserved
    volatile UINT16     RESERVED5;    ///< reserved
} tScCont;

/**
\brief Status/Control - Synchronization

The synchronization sub-register provide access to the synchronization features.
*/
typedef struct sScSync
{
    UINT16              irqEnable;       ///< enable IRQs
    volatile UINT16     irqPending;      ///< pending IRQs
    volatile UINT16     irqMasterEnable; ///< enable master IRQ
    union
    {
        volatile UINT16 set;             ///< set IRQ (PCP)
        volatile UINT16 ack;             ///< acknowledge IRQ (Host)
    } irq;
    volatile UINT32     RESERVED0;       ///< reserved
    volatile UINT16     syncConfig;      ///< synchronization configuration
    volatile UINT16     RESERVED1;       ///< reserved
} tScSync;

/**
\brief Status/Control - Dynamic Buffer Host side

The dynamic buffer's address registers seen from the Host side.
*/
typedef struct sScDynBufHost
{
    volatile UINT32      aDynBuf[HOSTIF_DYNBUF_COUNT];
        ///< base address array of dynamic buffers
} tScDynBufHost;

/**
\brief Status/Control - Dynamic Buffer PCP side

The dynamic buffer's address registers seen from the PCP side.
*/
typedef struct sScDynBufPcp
{
    volatile UINT32     aBuf[HOSTIF_BUF_COUNT];
        ///< base address array of buffers
} tScDynBufPcp;

/**
\brief Status/Control - Dynamic Buffer

The dynamic buffers of PCP and Host
*/
typedef union sScDynB
{
    volatile tScDynBufHost   Host;
    volatile tScDynBufPcp    Pcp;
} tScDynB;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static inline UINT32 hostifRead32(UINT8* pAddr_p, UINT offset_p);
static inline UINT16 hostifRead16(UINT8* pAddr_p, UINT offset_p);
static inline void hostifWrite32(UINT8* pAddr_p, UINT offset_p, UINT32 val_p);
static inline void hostifWrite16(UINT8* pAddr_p, UINT offset_p, UINT16 val_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Read magic word

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the magic word.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT32 hostif_readMagic(UINT8* pHostifScBase_p)
{
    return hostifRead32(pHostifScBase_p + HOSTIF_SC_INFO_OFFS,
                        offsetof(tScInfo, magic));
}

//------------------------------------------------------------------------------
/**
\brief  Read version

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the version field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT32 hostif_readVersion(UINT8* pHostifScBase_p)
{
    return hostifRead32(pHostifScBase_p + HOSTIF_SC_INFO_OFFS,
                        offsetof(tScInfo, version));
}

//------------------------------------------------------------------------------
/**
\brief  Read boot base

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the boot base address.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT32 hostif_readBootBase(UINT8* pHostifScBase_p)
{
    return hostifRead32(pHostifScBase_p + HOSTIF_SC_INFO_OFFS,
                        offsetof(tScInfo, bootBase));
}

//------------------------------------------------------------------------------
/**
\brief  Write boot base

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p              pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeBootBase(UINT8* pHostifScBase_p, UINT32 val_p)
{
    hostifWrite32(pHostifScBase_p + HOSTIF_SC_INFO_OFFS,
                  offsetof(tScInfo, bootBase), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read init base

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the initialization base address.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT32 hostif_readInitBase(UINT8* pHostifScBase_p)
{
    return hostifRead32(pHostifScBase_p + HOSTIF_SC_INFO_OFFS,
                        offsetof(tScInfo, initBase));
}

//------------------------------------------------------------------------------
/**
\brief  Write init base

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeInitBase(UINT8* pHostifScBase_p, UINT32 val_p)
{
    hostifWrite32(pHostifScBase_p + HOSTIF_SC_INFO_OFFS,
                  offsetof(tScInfo, initBase), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read bridge enable field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the bridge enable field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readBridgeEnable(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                        offsetof(tScCont, bridgeEnable));
}

//------------------------------------------------------------------------------
/**
\brief  Write bridge enabled field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeBridgeEnable(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                  offsetof(tScCont, bridgeEnable), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read command field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the command field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readCommand(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                        offsetof(tScCont, command));
}

//------------------------------------------------------------------------------
/**
\brief  Write command field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeCommand(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                  offsetof(tScCont, command), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read state field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the state field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readState(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                        offsetof(tScCont, state));
}

//------------------------------------------------------------------------------
/**
\brief  Write state field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeState(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                  offsetof(tScCont, state), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read return field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the return field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readReturn(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                        offsetof(tScCont, ret));
}

//------------------------------------------------------------------------------
/**
\brief  Write return field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeReturn(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                  offsetof(tScCont, ret), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read heart beat field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the hear beat field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readHeartbeat(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                        offsetof(tScCont, heartbeat));
}

//------------------------------------------------------------------------------
/**
\brief  Write heart beat field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeHeartbeat(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_CONT_OFFS,
                  offsetof(tScCont, heartbeat), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read irq enable field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the irq enable field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readIrqEnable(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                        offsetof(tScSync, irqEnable));
}

//------------------------------------------------------------------------------
/**
\brief  Write irq enable field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeIrqEnable(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                  offsetof(tScSync, irqEnable), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read irq pending field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the irq pending field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readIrqPending(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                        offsetof(tScSync, irqPending));
}

//------------------------------------------------------------------------------
/**
\brief  Read irq master enable field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the irq master enable field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readIrqMasterEnable(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                        offsetof(tScSync, irqMasterEnable));
}

//------------------------------------------------------------------------------
/**
\brief  Write irq master enable field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeIrqMasterEnable(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                  offsetof(tScSync, irqMasterEnable), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write irq acknowledge field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_ackIrq(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                  offsetof(tScSync, irq.ack), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write irq set field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p              pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_setIrq(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                  offsetof(tScSync, irq.set), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read synchronization configuration field

\param  pHostifScBase_p     base address of Status/Control registers

\return The function returns the irq synchronization configuration field.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT16 hostif_readSyncConfig(UINT8* pHostifScBase_p)
{
    return hostifRead16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                        offsetof(tScSync, syncConfig));
}

//------------------------------------------------------------------------------
/**
\brief  Write synchronization configuration field

\param  pHostifScBase_p     base address of Status/Control registers
\param  val_p               pattern to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeSyncConfig(UINT8* pHostifScBase_p, UINT16 val_p)
{
    hostifWrite16(pHostifScBase_p + HOSTIF_SC_SYNC_OFFS,
                  offsetof(tScSync, syncConfig), val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read dynamic buffer (Host only)

\param  pHostifScBase_p     base address of Status/Control registers
\param  num_p               determines the addressed dynamic buffer

\return The function returns the dynamic buffer address. Note that the returned
address is limited by the address width of the bridge master.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT32 hostif_readDynBufHost(UINT8* pHostifScBase_p, UINT8 num_p)
{
    return hostifRead32(pHostifScBase_p + HOSTIF_SC_DYNB_OFFS,
                        offsetof(tScDynB, Host.aDynBuf[num_p]));
}

//------------------------------------------------------------------------------
/**
\brief  Write dynamic buffer (Host only)

\param  pHostifScBase_p     base address of Status/Control registers
\param  num_p               determines the addressed dynamic buffer
\param  addr_p              address to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeDynBufHost(UINT8* pHostifScBase_p, UINT8 num_p, UINT32 addr_p)
{
    hostifWrite32(pHostifScBase_p + HOSTIF_SC_DYNB_OFFS,
                  offsetof(tScDynB, Host.aDynBuf[num_p]), addr_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read buffer (PCP only)

\param  pHostifScBase_p     base address of Status/Control registers
\param  num_p               determines the addressed buffer

\return The function returns the dynamic buffer address. Note that the returned
address is limited by the address width of the bridge master.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
UINT32 hostif_readBufPcp(UINT8* pHostifScBase_p, UINT8 num_p)
{
    return hostifRead32(pHostifScBase_p + HOSTIF_SC_DYNB_OFFS,
                        offsetof(tScDynB, Pcp.aBuf[num_p]));
}

//------------------------------------------------------------------------------
/**
\brief  Write dynamic buffer (PCP only)

\param  pHostifScBase_p     base address of Status/Control registers
\param  num_p               determines the addressed buffer
\param  addr_p              address to be written

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_writeBufPcp(UINT8* pHostifScBase_p, UINT8 num_p, UINT32 addr_p)
{
    hostifWrite32(pHostifScBase_p + HOSTIF_SC_DYNB_OFFS,
                  offsetof(tScDynB, Pcp.aBuf[num_p]), addr_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Read 32 bit value

The function reads from the given base address plus the provided offset the
32 bit value.

\param  pAddr_p     Base address
\param  offset_p    Offset from given base address

\return The function returns the read value.
*/
//------------------------------------------------------------------------------
static UINT32 hostifRead32(UINT8* pAddr_p, UINT offset_p)
{
    HOSTIF_DCACHE_INVALIDATE(pAddr_p + offset_p, sizeof(UINT32));

    return HOSTIF_RD32(pAddr_p + offset_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read 16 bit value

The function reads from the given base address plus the provided offset the
16 bit value.

\param  pAddr_p     Base address
\param  offset_p    Offset from given base address

\return The function returns the read value.
*/
//------------------------------------------------------------------------------
static UINT16 hostifRead16(UINT8* pAddr_p, UINT offset_p)
{
    HOSTIF_DCACHE_INVALIDATE(pAddr_p + offset_p, sizeof(UINT16));

    return HOSTIF_RD16(pAddr_p + offset_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write 32 bit value

The function writes to the given base address plus the provided offset the given
32 bit value.

\param  pAddr_p     Base address
\param  offset_p    Offset from given base address
\param  val_p       Value to be written to the given base plus offset
*/
//------------------------------------------------------------------------------
static void hostifWrite32(UINT8* pAddr_p, UINT offset_p, UINT32 val_p)
{
    HOSTIF_WR32(pAddr_p + offset_p, val_p);

    HOSTIF_DCACHE_FLUSH(pAddr_p + offset_p, sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Write 16 bit value

The function writes to the given base address plus the provided offset the given
16 bit value.

\param  pAddr_p     Base address
\param  offset_p    Offset from given base address
\param  val_p       Value to be written to the given base plus offset
*/
//------------------------------------------------------------------------------
static void hostifWrite16(UINT8* pAddr_p, UINT offset_p, UINT16 val_p)
{
    HOSTIF_WR32(pAddr_p + offset_p, val_p);

    HOSTIF_DCACHE_FLUSH(pAddr_p + offset_p, sizeof(UINT16));
}

/// \}
