/*
 * Copyright (c) 2010 Avnet.  All rights reserved.
 *
 * Avnet (HK)
 * AVNET IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, AVNET IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION
 * AVNET EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "xbasic_types.h"
#include "xstatus.h"
#include "global.h"

#if BIT_SWAP_ON == 1
u8 BitReOrder8(u8 data)
{
    u8 dswap = data;

    dswap = ((dswap&0x1)<<7) | ((dswap&0x2)<<5) | ((dswap&0x4)<<3) | ((dswap&0x8)<<1) | \
                ((dswap&0x10)>>1) | ((dswap&0x20)>>3) | ((dswap&0x40)>>5) | ((dswap&0x80)>>7);

    return dswap;
}
#endif

u16 BitReOrder16(Xuint16 data)
{
    Xuint8 dhi, dlo;

#if SWAP_HILO_BYTE == 1
    /* swap hi-lo byte */
    dlo = (Xuint8)(data >> 8);
    dhi = (Xuint8)data;
#else
    /* no need to swap hi-lo byte */
    dlo = (Xuint8)data;
    dhi = (Xuint8)(data >> 8);
#endif

#if BIT_SWAP_ON == 1
    /* swap bits by software */
    dlo = BitReOrder8(dlo);
    dhi = BitReOrder8(dhi);
#endif

    return (dhi << 8) | dlo;
}

