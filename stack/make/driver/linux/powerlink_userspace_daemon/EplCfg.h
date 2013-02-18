/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  configuration file

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
                    ...

  -------------------------------------------------------------------------

  Revision History:

  2006/06/06    k.t.: Start of Implementation

****************************************************************************/

#ifndef _EPLCFG_H_
#define _EPLCFG_H_

#ifdef __linux__
/* assure that system priorities of hrtimer and net-rx kernel threads are set appropriate */
#define EPL_THREAD_PRIORITY_HIGH     75
#define EPL_THREAD_PRIORITY_MEDIUM   50
#define EPL_THREAD_PRIORITY_LOW      49

#undef FTRACE_DEBUG
#endif // __linux__

// =========================================================================
// generic defines which for whole EPL Stack
// =========================================================================
#define EPL_USE_DELETEINST_FUNC TRUE

// needed to support datatypes over 32 bit by global.h
#define USE_VAR64

// EPL_MAX_INSTANCES specifies count of instances of all EPL modules.
// If it is greater than 1 the first parameter of all
// functions is the instance number.
#define EPL_MAX_INSTANCES               1


#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES       0 //0xEE800042L
#endif

// Default debug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
//#define DEF_DEBUG_LVL           0xEC000084L
#define DEF_DEBUG_LVL           (0xC00000000L)
#endif

#ifdef CONFIG_MN
#define EPL_MODULE_INTEGRATION (EPL_MODULE_DLLK \
                                | EPL_MODULE_PDOK \
                                | EPL_MODULE_NMTK \
                                | EPL_MODULE_NMT_MN)
#else
#define EPL_MODULE_INTEGRATION (EPL_MODULE_DLLK \
                                | EPL_MODULE_PDOK \
                                | EPL_MODULE_NMTK \
                                | EPL_MODULE_NMT_CN)
#endif

// =========================================================================
// EPL ethernet driver (Edrv) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
#define EDRV_FAST_TXFRAMES              FALSE
//#define EDRV_FAST_TXFRAMES              TRUE

// switch this define to TRUE if Edrv supports early receive interrupts
#define EDRV_EARLY_RX_INT               FALSE
//#define EDRV_EARLY_RX_INT               TRUE

// enables setting of several port pins for benchmarking purposes
#define EDRV_BENCHMARK                  FALSE
//#define EDRV_BENCHMARK                  TRUE // MCF_GPIO_PODR_PCIBR

// Call Tx handler (i.e. EplDllCbFrameTransmitted()) already if DMA has finished,
// otherwise call the Tx handler if frame was actually transmitted over ethernet.
#define EDRV_DMA_TX_HANDLER             FALSE
//#define EDRV_DMA_TX_HANDLER             TRUE

// number of used ethernet controller
//#define EDRV_USED_ETH_CTRL              1

#define EDRV_USE_DIAGNOSTICS            TRUE

// =========================================================================
// Data Link Layer (DLL) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoC
#define EPL_DLL_PRES_READY_AFTER_SOC    FALSE
//#define EPL_DLL_PRES_READY_AFTER_SOC    TRUE

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoA
#define EPL_DLL_PRES_READY_AFTER_SOA    FALSE
//#define EPL_DLL_PRES_READY_AFTER_SOA    TRUE

// activate PResChaining support on MN
#define EPL_DLL_PRES_CHAINING_MN        TRUE

// Disable deferred release of rx-buffers until EdrvPcap supports it
#define EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE    TRUE

// =========================================================================
// Timer module specific defines
// =========================================================================

// if TRUE the high resolution timer module will be used
#define EPL_TIMER_USE_HIGHRES              TRUE
//#define EPL_TIMER_USE_HIGHRES              FALSE

#endif //_EPLCFG_H_
