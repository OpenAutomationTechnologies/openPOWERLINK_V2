/* omethlibint.h - Ethernet Library for FPGA MAC Controller */
/*
------------------------------------------------------------------------------
Copyright (c) 2009, B&R
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

- Neither the name of the B&R nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

------------------------------------------------------------------------------
 Module:    omethlibint
 File:      omethlibint.h
 Author:    Thomas Enzinger(enzingert)
 Created:   05.10.2006
 Revised:   25.01.2010
 State:     tested on Altera Nios II and Xilinx Microblaze
------------------------------------------------------------------------------

 Functions:
 omethFilterDisable            - disable filter
 omethFilterSetByteValue       - change 1 filter value byte of a filter

 omethGetTimestamp             - get time stamp of a received packet

 omethRxIrqHandler             - irq handler for packet receive
 omethTxIrqHandler             - irq handler for packet transmit

------------------------------------------------------------------------------
 History:
    05.10.2006    enzinger    created
    21.08.2009    zelenkaj    revised
    24.10.2010    zelenkaj    added rx/tx buffer bases into eth drv instance
----------------------------------------------------------------------------*/

#include <omethlib.h>
#include <omethlibint.h>
#include <string.h>                // used functions: memcpy, memset
#include <stdlib.h>                // used functions: calloc

/*****************************************************************************
*
* omethFilterDisable - disable filter
*
*/
void                omethFilterDisable
(
 OMETH_FILTER_H    hFilter
)
{
    // disable filter
    *hFilter->pFilterData->pCommand = hFilter->pFilterData->cmd &= CMD_FILTER_nON;
}

/*****************************************************************************
*
* omethFilterSetByteValue - sets one byte (value, not mask) to an filter
* and enable filter
*
*    ! the function does not disable the filter while changing the value
*        ->    if more than 1 byte should be changed consistent omethFilterSetPattern()
*            shall be used instead
*
*/
void            omethFilterSetByteValue
(
 OMETH_FILTER_H    hFilter,    /* filter handle                                    */
 unsigned short    offset,        /* offset in the filterarray                        */
 unsigned char    value        /* value to set                                        */
)
{
    hFilter->pFilterData->pFilterWriteOnly[offset].value = value;
//    FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);
}

/*****************************************************************************
*
* omethRxIrqHandler - to be called from the MAC RX interrupt
*
*/
void            omethRxIrqHandler
(
 OMETH_H        hEth        /* handle of ethernet driver, see omethCreate() */
 )
{
    // Timing measurement on OM32 IP20 Eval Board
    //
    // IRQ for hook with maxPending==0 and empty hook : 3,5 us
    // IRQ for hook with maxPending==x and empty hook : 5 us    (4 us if no buffer available)

    OMETH_HOOK_H              hHook;
    OMETH_FILTER_H            hFilter;
    ometh_buf_typ             *pRxBuf;
    ometh_pending_typ         *pQueue;                    // ptr to buffer queue
    ometh_rx_info_typ         *pInfo;
    ometh_desc_typ            *pDesc;
    unsigned long             flags=0;

    #ifdef DEBUG_OUTPUT_ETH_RX_HOOK_SET
        DEBUG_OUTPUT_ETH_RX_HOOK_SET();
    #endif

    pInfo = hEth->pRxNext;    // access to next rx info structure
    pDesc = pInfo->pDesc;    // access to rx descriptor

    if(pDesc->flags.byte.high & FLAGS1_OWNER) return;    // leave IRQ if no rx-buffer available

    // !! here hHook is abused to save stack-variables (until required for its original reason)
    //    (faster processing)
    hHook = (OMETH_HOOK_H)&hEth->pRegBase->rxStatus;
    ((ometh_status_typ*)hHook)->clrBit = OMETH_REG_IQUIT;

    flags = pDesc->flags.word;

    // access to buffer structure and set packet length
    pRxBuf = GET_TYPE_BASE( ometh_buf_typ , packet.data, pDesc->pData);

    pRxBuf->packet.length = (unsigned long)pDesc->len - 4;    // length without checksum
    pRxBuf->timeStamp     = pDesc->time;                    // overtake timestamp to packet header

    if(((ometh_status_typ*)hHook)->value & OMETH_REG_LOST)
    {
        ((ometh_status_typ*)hHook)->clrBit = OMETH_REG_LOST;

        hEth->stat.rxLost++;
    }

    #ifdef DEBUG_OUTPUT_ETH_RX_HOOK_CLR
        DEBUG_OUTPUT_ETH_RX_HOOK_CLR();
    #endif

    if( (flags & (ETH_FLAGS_CRC_ERROR|ETH_FLAGS_OVERSIZE)) == 0 )
    {
        hEth->stat.rxOk++;

        hFilter    = hEth->pFilterList + ((flags & 0xF0) >> 4);        // access to filter info structure
        hHook    = hFilter->hHook;        // access to hook function of this filter

        if ((hFilter->pFilterData->cmd & CMD_FILTER_ON) == 0)
        {
            hEth->stat.rxHookDisabled++;
        }

        if(hHook->pFct)
        {
            if(hHook->pFreeRead==0)
            {
                #ifdef DEBUG_OUTPUT_ETH_RX_HOOK_SET
                    DEBUG_OUTPUT_ETH_RX_HOOK_SET();
                #endif

                // call hook function (no release function is passed because hook is not allowed to queue this frame)
                hHook->pFct(hFilter->arg, &pRxBuf->packet, 0);

                #ifdef DEBUG_OUTPUT_ETH_RX_HOOK_CLR
                    DEBUG_OUTPUT_ETH_RX_HOOK_CLR();
                #endif
            }
            else
            {
                pRxBuf->hHook = hHook;                // overtake hook handle for free function

                pQueue = hHook->pFreeRead;

                if(pQueue->pBuf)        // buffer available for exchange
                {
                    #ifdef DEBUG_OUTPUT_ETH_RX_HOOK_SET
                        DEBUG_OUTPUT_ETH_RX_HOOK_SET();
                    #endif

                    // call hook function
                    if(hHook->pFct(hFilter->arg, &pRxBuf->packet, omethPacketFree) == 0)
                    {
                        // use new frame for next rx at this descriptor
                        pDesc->pData = (unsigned long)&pQueue->pBuf->packet.data;

                        pQueue->pBuf = 0;                    // remove buffer from list

                        hHook->pFreeRead = pQueue->pNext;    // switch to next buffer in queue
                    }

                    #ifdef DEBUG_OUTPUT_ETH_RX_HOOK_CLR
                        DEBUG_OUTPUT_ETH_RX_HOOK_CLR();
                    #endif
                }
                else
                {
                    hHook->cntOverflow++;            // too many buffers pending, buffer is not passed to the client
                    hEth->stat.rxHookOverflow++;    // also count in user accessible statistic structure
                }
            }
        }
        else
        {
            hEth->stat.rxHookDisabled++;
        }
    }
    else
    {
        if(flags & ETH_FLAGS_OVERSIZE)    hEth->stat.rxOversize++;
        if(flags & ETH_FLAGS_CRC_ERROR)    hEth->stat.rxCrcError++;
    }

    // pass buffer to MAC
    pDesc->len                = hEth->rxLen;        // set maximum length for this buffer
    pDesc->flags.byte.high    = pInfo->flags1;    // set owner and last flag
    hEth->pRxNext            = pInfo->pNext;        // switch to next info for next rx
}

/*****************************************************************************
*
* omethTxIrqHandler - to be called from the MAC TX interrupt
*
*/
void            omethTxIrqHandler
(
 OMETH_H        hEth        /* handle of ethernet driver, see omethCreate() */
 )
{
    ometh_tx_info_typ    *pInfo    = hEth->pTxFree[0];    // access to next tx info structure;
    ometh_desc_typ        *pDesc    = pInfo->pDesc;        // access to tx descriptor
    unsigned long        i;


#if (OMETH_ENABLE_SOFT_IRQ==1)
    // check if soft IRQ was triggered
    if(hEth->pRegBase->txStatus.value & OMETH_REG_SOFTIRQ)
    {
        hEth->pRegBase->txStatus.clrBit = OMETH_REG_SOFTIRQ;

        hEth->pFctSoftIrq(0);        // call user function for software IRQ

        return;
    }
#endif

    // return if no tx irq pending on this interface
    if((hEth->pRegBase->txStatus.value & OMETH_REG_PENDING) == 0) return;

    while(1)
    {
        // if it was not the last queue-descriptor .. it was an auto answer buffer
        if(pDesc->flags.byte.high & FLAGS1_SENT)
        {
            i = (unsigned long)pDesc->flags.word & 0x0F;    // collisions of this frame
            hEth->stat.txDone[i]++;                            // count transmits depending on occurred collisions
            hEth->stat.txCollision += i;                    // count collisions

            // call free function with ptr
            if(pInfo->pFctFree)
            {
                pInfo->pFctFree( GET_TYPE_BASE(ometh_packet_typ, data, pDesc->pData), pInfo->fctFreeArg, pDesc->time );
            }

            hEth->pTxFree[0] = pInfo->pNext;    // switch free ptr to next info structure

            hEth->cntTxQueueOut++;

            pDesc->flags.byte.high    = 0;                // clear sent-flag
            pDesc->pData            = 0;                  // mark buffer as free
            break;
        }

        pInfo = hEth->pTxFree[1];    // try second priority queue

        if(pInfo) // check if 2nd transmit queue existing
        {
            pDesc    = pInfo->pDesc;        // access to tx descriptor

            // if it was not the last queue-descriptor .. it was an auto answer buffer
            if(pDesc->flags.byte.high & FLAGS1_SENT)
            {
                i = (unsigned long)pDesc->flags.word & 0x0F;    // collisions of this frame
                hEth->stat.txDone[i]++;                         // count transmits depending on occurred collisions
                hEth->stat.txCollision += i;                    // count collisions

                // call free function with ptr
                if(pInfo->pFctFree)
                {
                    pInfo->pFctFree( GET_TYPE_BASE(ometh_packet_typ, data, pDesc->pData), pInfo->fctFreeArg, pDesc->time );
                }

                hEth->pTxFree[1] = pInfo->pNext;    // switch free ptr to next info structure

                hEth->cntTxQueueOut++;

                pDesc->flags.byte.high    = 0;                // clear sent-flag
                pDesc->pData            = 0;                  // mark buffer as free
                break;
            }
        }

        // search through all auto transmit descriptors to find the one causing the irq
        pInfo = hEth->pTxAuto;
        while(1)
        {
            if(pInfo->flags1 & FLAGS1_LAST)    // this is already the last-descriptor of the queue
            {

                #ifdef DEBUG_OUTPUT_ETH_SPURIOUS_SET
                    DEBUG_OUTPUT_ETH_SPURIOUS_SET();
                #endif

                // return if no sent buffer found
                hEth->stat.txSpuriousInt++;

                hEth->pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;    // quit tx irq

                #ifdef DEBUG_OUTPUT_ETH_SPURIOUS_CLR
                    DEBUG_OUTPUT_ETH_SPURIOUS_CLR();
                #endif

                return;
            }

            pDesc = pInfo->pDesc;                    // access to descriptor

            if(pDesc->flags.byte.high & FLAGS1_SENT) break;    // descriptor found, break loop

            pInfo = pInfo->pNext;                    // switch to next tx info
        }

        pInfo->autoTxCount++;

        i = (unsigned long)pDesc->flags.word & 0x0F;    // collisions of this frame
        hEth->stat.txDone[i]++;                            // count transmits depending on occurred collisions
        hEth->stat.txCollision += i;                    // count collisions

        // set owner flag for next auto tx
        pDesc->txStart            = pInfo->delayTime;
        pDesc->flags.byte.high    = FLAGS1_OWNER | FLAGS1_TX_DELAY;
        break;
    }

    hEth->pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;    // quit tx irq
}

/*****************************************************************************
*
* omethGetTimestamp - get timestamp of a received packet
*
*/
unsigned long    omethGetTimestamp
(
 ometh_packet_typ    *pPacket    /* address of rx packet*/
 )
{
    if(pPacket==0) return 0;

    return GET_TYPE_BASE( ometh_buf_typ, packet, pPacket)->timeStamp;
}
