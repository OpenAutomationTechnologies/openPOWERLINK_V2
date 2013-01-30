/****************************************************************************

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com


  Project:      openPOWERLINK

  Description:  Ethernet Driver definition for openMAC with XPS PLB

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

#ifndef __EDRV_OPENMAC_XPS_AXI_H_
#define __EDRV_OPENMAC_XPS_AXI_H_

#include "xparameters.h"

// DEFINES FOR EDRV AND EDRVCYCLIC
//--- packet location definitions ---
#define EDRV_PKT_LOC_TX_RX_INT                0
#define EDRV_PKT_LOC_TX_INT_RX_EXT            1
#define EDRV_PKT_LOC_TX_RX_EXT                2

#define EDRV_INTC_BASE                  XPAR_PCP_INTC_BASEADDR
#define EDRV_MAC_BASE           (void *)XPAR_PLB_POWERLINK_0_MAC_REG_BASEADDR
#define EDRV_MAC_SPAN                   (XPAR_PLB_POWERLINK_0_MAC_REG_HIGHADDR-XPAR_PLB_POWERLINK_0_MAC_REG_BASEADDR+1)
#define EDRV_MAC_IRQ                    XPAR_PCP_INTC_PLB_POWERLINK_0_MAC_IRQ_INTR
#define EDRV_MAC_IRQ_MASK               XPAR_PLB_POWERLINK_0_MAC_IRQ_MASK
#define EDRV_RAM_BASE           (void *)(EDRV_MAC_BASE + 0x0800)
#define EDRV_MII_BASE           (void *)(EDRV_MAC_BASE + 0x1000)
#define EDRV_IRQ_BASE           (void *)(EDRV_MAC_BASE + 0x1010)
#define EDRV_DOB_BASE           (void *)(EDRV_MAC_BASE + 0x1020)
#define EDRV_CMP_BASE           (void *)XPAR_PLB_POWERLINK_0_MAC_CMP_BASEADDR
#define EDRV_CMP_SPAN                   (XPAR_PLB_POWERLINK_0_MAC_CMP_HIGHADDR-XPAR_PLB_POWERLINK_0_MAC_CMP_BASEADDR+1)
#define EDRV_PKT_LOC                    XPAR_PLB_POWERLINK_0_PACKET_LOCATION
#define EDRV_PHY_NUM                    XPAR_PLB_POWERLINK_0_PHY_COUNT
#define EDRV_DMA_OBSERVER               XPAR_PLB_POWERLINK_0_OBSERVER_ENABLE
#define EDRV_MAX_RX_BUFFERS             XPAR_PLB_POWERLINK_0_MAC_RX_BUFFERS
#if EDRV_PKT_LOC == EDRV_PKT_LOC_TX_RX_INT
    #define EDRV_PKT_BASE           (void *)XPAR_PLB_POWERLINK_0_MAC_PKT_BASEADDR
    #define EDRV_PKT_SPAN                   XPAR_PLB_POWERLINK_0_MAC_PKT_SIZE
#elif EDRV_PKT_LOC == EDRV_PKT_LOC_TX_RX_EXT
    #define EDRV_PKT_BASE           (void *)0 //not used
    #define EDRV_PKT_SPAN                   0 //not used
#elif EDRV_PKT_LOC == EDRV_PKT_LOC_TX_INT_RX_EXT
    #define EDRV_PKT_BASE           (void *)XPAR_PLB_POWERLINK_0_MAC_PKT_BASEADDR
    #define EDRV_PKT_SPAN                   XPAR_PLB_POWERLINK_0_MAC_PKT_SIZE
#endif

// DEFINES FOR Interrupt Controller
#define EPL_TIMER_INTC_BASE        XPAR_PCP_INTC_BASEADDR

// DEFINES FOR TIMERSYNC
#define EPL_TIMER_SYNC_BASE        XPAR_PLB_POWERLINK_0_MAC_CMP_BASEADDR
#define EPL_TIMER_SYNC_IRQ         XPAR_PCP_INTC_PLB_POWERLINK_0_TCP_IRQ_INTR
#define EPL_TIMER_SYNC_IRQ_MASK    XPAR_PLB_POWERLINK_0_TCP_IRQ_MASK
#if XPAR_PLB_POWERLINK_0_PDI_GEN_TIME_SYNC != FALSE
    #define EPL_TIMER_USE_COMPARE_PDI_INT
#endif

// DEFINES FOR TIMERHIGHRES
#define HIGHRES_TIMER_IRQ           XPAR_PCP_INTC_PLB_POWERLINK_0_TCP_IRQ_INTR
#define HIGHRES_TIMER_IRQ_MASK      XPAR_PLB_POWERLINK_0_TCP_IRQ_MASK
#define HIGHRES_TIMER_BASE          XPAR_PLB_POWERLINK_0_MAC_CMP_BASEADDR

#endif
