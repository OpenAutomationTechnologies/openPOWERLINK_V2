/****************************************************************************

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com


  Project:      openPOWERLINK

  Description:  Ethernet Driver definition for openMAC in Qsys "pcp_0"

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------
                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

----------------------------------------------------------------------------*/

#ifndef __EDRV_OPENMAC_QSYS_H_
#define __EDRV_OPENMAC_QSYS_H_

#include "system.h"

// DEFINES FOR EDRV AND EDRVCYCLIC
//--- packet location definitions ---
#define EDRV_PKT_LOC_TX_RX_INT                0
#define EDRV_PKT_LOC_TX_INT_RX_EXT            1
#define EDRV_PKT_LOC_TX_RX_EXT                2

#define EDRV_MAC_BASE           (void *)PCP_0_POWERLINK_0_MAC_REG_BASE
#define EDRV_MAC_SPAN                   PCP_0_POWERLINK_0_MAC_REG_SPAN
#define EDRV_MAC_IRQ                    PCP_0_POWERLINK_0_MAC_REG_IRQ
#define EDRV_MAC_IRQ_IC_ID              PCP_0_POWERLINK_0_MAC_REG_IRQ_INTERRUPT_CONTROLLER_ID
#define EDRV_RAM_BASE           (void *)(EDRV_MAC_BASE + 0x0800)
#define EDRV_MII_BASE           (void *)(EDRV_MAC_BASE + 0x1000)
#define EDRV_IRQ_BASE           (void *)(EDRV_MAC_BASE + 0x1010)
#define EDRV_DOB_BASE           (void *)(EDRV_MAC_BASE + 0x1020)
#define EDRV_CMP_BASE           (void *)PCP_0_POWERLINK_0_MAC_CMP_BASE
#define EDRV_CMP_SPAN                   PCP_0_POWERLINK_0_MAC_CMP_SPAN
#define EDRV_PKT_LOC                    PCP_0_POWERLINK_0_MAC_REG_PKTLOC
#define EDRV_PHY_NUM                    PCP_0_POWERLINK_0_MAC_REG_PHYCNT
#define EDRV_DMA_OBSERVER               PCP_0_POWERLINK_0_MAC_REG_DMAOBSERV
#define EDRV_MAX_RX_BUFFERS             PCP_0_POWERLINK_0_MAC_REG_MACRXBUFFERS

#if EDRV_PKT_LOC == EDRV_PKT_LOC_TX_RX_INT                        //TX+RX in M9K
    #define EDRV_PKT_BASE           (void *)PCP_0_POWERLINK_0_MAC_BUF_BASE
    #define EDRV_PKT_SPAN                   PCP_0_POWERLINK_0_MAC_BUF_MACBUFSIZE
#elif EDRV_PKT_LOC == EDRV_PKT_LOC_TX_INT_RX_EXT                        //TX in M9K and RX in external memory
    #define EDRV_PKT_BASE           (void *)PCP_0_POWERLINK_0_MAC_BUF_BASE
    #define EDRV_PKT_SPAN                   PCP_0_POWERLINK_0_MAC_BUF_MACBUFSIZE
#elif EDRV_PKT_LOC == EDRV_PKT_LOC_TX_RX_EXT                        //TX+RX in external memory
    #define EDRV_PKT_BASE           (void *)0 //not used
    #define EDRV_PKT_SPAN                   0 //not used
#endif

// DEFINES FOR TIMERSYNC
#define EPL_TIMER_SYNC_BASE         PCP_0_POWERLINK_0_MAC_CMP_BASE //from system.h
#define EPL_TIMER_SYNC_IRQ          PCP_0_POWERLINK_0_MAC_CMP_IRQ
#define EPL_TIMER_SYNC_IRQ_IC_ID    PCP_0_POWERLINK_0_MAC_CMP_IRQ_INTERRUPT_CONTROLLER_ID
#if PCP_0_POWERLINK_0_MAC_CMP_TIMESYNCHW != FALSE
    #define EPL_TIMER_USE_COMPARE_PDI_INT
#endif

// DEFINES FOR TIMERHIGHRES
#define HIGHRES_TIMER_IRQ           PCP_0_POWERLINK_0_MAC_CMP_IRQ
#define HIGHRES_TIMER_IRQ_IC_ID     PCP_0_POWERLINK_0_MAC_CMP_IRQ_INTERRUPT_CONTROLLER_ID
#define HIGHRES_TIMER_BASE          PCP_0_POWERLINK_0_MAC_CMP_BASE

#endif
