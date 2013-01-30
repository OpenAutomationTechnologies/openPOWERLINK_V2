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

****************************************************************************/

#ifndef _EPLCFG_H_
#define _EPLCFG_H_

// =========================================================================
// generic defines which for whole EPL Stack
// =========================================================================
#define EPL_USE_DELETEINST_FUNC         TRUE

// needed to support datatypes over 32 bit by global.h
#define USE_VAR64

// EPL_MAX_INSTANCES specifies count of instances of all EPL modules.
// If it is greater than 1 the first parameter of all
// functions is the instance number.
#define EPL_MAX_INSTANCES               1

// This defines the target hardware. Here is encoded wich CPU and wich external
// peripherals are connected. For possible values refere to target.h. If
// necessary value is not available EPL stack has to
// be adapted and tested.
#define TARGET_HARDWARE                 TGTHW_PC_WRAPP

// use no FIFOs, make direct calls
//#define EPL_USE_SHAREDBUFF   FALSE

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES               0 //0xEE800042L
#endif

// Default debug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                   (0xC00000000L)
#endif

//#define EPL_DBGLVL_ASSERT               DEBUG_LVL_ASSERT    // 0x20000000L
//#define EPL_DBGLVL_ERROR                DEBUG_LVL_ERROR     // 0x40000000L
//#define EPL_DBGLVL_ALWAYS               DEBUG_LVL_ALWAYS    // 0x80000000L

// enable CFM module depending on CONFIG_CFM
#ifdef CONFIG_CFM
#define EPL_MODULE_CFM_CONFIG EPL_MODULE_CFM
#else
#define EPL_MODULE_CFM_CONFIG 0
#endif

// EPL_MODULE_INTEGRATION defines all modules which are included in
// EPL application. Please add or delete modules for your application.
#ifdef CONFIG_MN
#define EPL_MODULE_INTEGRATION (EPL_MODULE_OBDK \
                               | EPL_MODULE_PDOK \
                               | EPL_MODULE_PDOU \
                               | EPL_MODULE_SDOS \
                               | EPL_MODULE_SDOC \
                               | EPL_MODULE_SDO_ASND \
                               | EPL_MODULE_SDO_UDP \
                               | EPL_MODULE_NMT_CN \
                               | EPL_MODULE_NMTU \
                               | EPL_MODULE_NMTK \
                               | EPL_MODULE_DLLK \
                               | EPL_MODULE_DLLU \
                               | EPL_MODULE_CFM_CONFIG \
                               | EPL_MODULE_NMT_MN \
                               | EPL_MODULE_VETH)

#else

#define EPL_MODULE_INTEGRATION (EPL_MODULE_OBDK \
                               | EPL_MODULE_PDOK \
                               | EPL_MODULE_PDOU \
                               | EPL_MODULE_SDOS \
                               | EPL_MODULE_SDOC \
                               | EPL_MODULE_SDO_ASND \
                               | EPL_MODULE_SDO_UDP \
                               | EPL_MODULE_NMT_CN \
                               | EPL_MODULE_NMTU \
                               | EPL_MODULE_NMTK \
                               | EPL_MODULE_DLLK \
                               | EPL_MODULE_DLLU \
                               | EPL_MODULE_VETH)

#endif // CONFIG_MN

// =========================================================================
// EPL ethernet driver (Edrv) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
#define EDRV_FAST_TXFRAMES                  FALSE

// switch this define to TRUE if Edrv supports early receive interrupts
#define EDRV_EARLY_RX_INT                   FALSE

// enables setting of several port pins for benchmarking purposes
#define EDRV_BENCHMARK                      FALSE

// Call Tx handler (i.e. EplDllCbFrameTransmitted()) already if DMA has finished,
// otherwise call the Tx handler if frame was actually transmitted over ethernet.
#define EDRV_DMA_TX_HANDLER                 FALSE

// number of used ethernet controller
//#define EDRV_USED_ETH_CTRL                1

// increase the number of Tx buffers, because we are master
// and need one Tx buffer for each PReq and CN
// + SoC + SoA + MN PRes + NmtCmd + ASnd + IdentRes + StatusRes.
#define EDRV_MAX_TX_BUFFERS             80

#if (CONFIG_EDRV == 82573)
#define EDRV_USE_DIAGNOSTICS            TRUE
#endif

// =========================================================================
// Data Link Layer (DLL) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoC
#define EPL_DLL_PRES_READY_AFTER_SOC        FALSE

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoA
#define EPL_DLL_PRES_READY_AFTER_SOA        FALSE

#ifdef CONFIG_MN

// activate PResChaining support on MN
#define EPL_DLL_PRES_CHAINING_MN            TRUE

// CN supports PRes Chaining
#define EPL_DLL_PRES_CHAINING_CN            FALSE

#else

// activate PResChaining support on MN
#define EPL_DLL_PRES_CHAINING_MN            FALSE

// CN supports PRes Chaining
#define EPL_DLL_PRES_CHAINING_CN            FALSE

// negative time shift of isochronous task in relation to SoC
#define EPL_DLL_SOC_SYNC_SHIFT_US           150

#define EPL_DLL_PROCESS_SYNC                EPL_DLL_PROCESS_SYNC_ON_SOC

#define EDRV_AUTO_RESPONSE_DELAY            TRUE
#endif

#if (CONFIG_EDRV == 8139) || (CONFIG_EDRV == 8255)
// Disable deferred release of rx-buffers until Edrv8139/Edrv8255x supports it
#define EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE    TRUE
#endif

// =========================================================================
// OBD specific defines
// =========================================================================

// switch this define to TRUE if Epl should compare object range
// automaticly
#define EPL_OBD_CHECK_OBJECT_RANGE          FALSE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
#define EPL_OBD_USE_STRING_DOMAIN_IN_RAM    TRUE

#define EPL_OBD_USE_VARIABLE_SUBINDEX_TAB   TRUE

#ifdef CONFIG_CFM

#define EPL_OBD_USE_LOAD_CONCISEDCF         TRUE
#define EPL_OBD_DEF_CONCISEDCF_FILENAME     "mnobd.cdc"

#define EPL_CFM_CONFIGURE_CYCLE_LENGTH      TRUE

// Configure if the range from 0xA000 is used for
// mapping client objects.
// openCONFIGURATOR uses this range for mapping
// objects.
#ifdef CONFIG_OPENCONFIGURATOR_MAPPING
#define EPL_OBD_INCLUDE_A000_TO_DEVICE_PART TRUE
#endif

#else // CONFIG_CFM

#define EPL_OBD_USE_LOAD_CONCISEDCF         FALSE
#define EPL_CFM_CONFIGURE_CYCLE_LENGTH      FALSE
#define EPL_OBD_INCLUDE_A000_TO_DEVICE_PART FALSE

#endif // CONFIG_CFM

// configure whether OD access events shall be forwarded
// to user callback function.
// Because of reentrancy for local OD accesses, this has to be disabled
#define EPL_API_OBD_FORWARD_EVENT       FALSE

// =========================================================================
// Timer module specific defines
// =========================================================================

// if TRUE the high resolution timer module will be used
#define EPL_TIMER_USE_HIGHRES               TRUE

// =========================================================================
// SDO module specific defines
// =========================================================================

#ifdef CONFIG_MN

// increase the number of SDO channels, because we are master
#define EPL_SDO_MAX_CONNECTION_ASND         100
#define EPL_MAX_SDO_SEQ_CON                 100
#define EPL_MAX_SDO_COM_CON                 100
#define EPL_SDO_MAX_CONNECTION_UDP          50

#endif

#endif //_EPLCFG_H_
