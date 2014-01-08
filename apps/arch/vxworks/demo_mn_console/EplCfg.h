/****************************************************************************
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, A-5142 Eggelsberg
      www.br-automation.com

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

    3. Neither the name of the copyright holders nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

#define EPL_TASK_PRIORITY_HRTIMER       1
#define EPL_TASK_PRIORITY_NETTX         1
#define EPL_TASK_PRIORITY_SHB           10
#define EPL_TASK_PRIORITY_UTIMER        50
#define EPL_TASK_PRIORITY_SHB_JOB	50
#define EPL_TASK_PRIORITY_SDOUDP        50
#define EPL_TASK_STACK_SIZE             32768

// =========================================================================
// generic defines which for whole EPL Stack
// =========================================================================
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

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES               0 //0xEE800042L
#endif

// Default defug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                   0xEC000080L
#endif
//   EPL_DBGLVL_OBD         =   0x00000004L
// * EPL_DBGLVL_ASSERT      =   0x20000000L
// * EPL_DBGLVL_ERROR       =   0x40000000L
// * EPL_DBGLVL_ALWAYS      =   0x80000000L

// These macros define all modules which are included
#define CONFIG_INCLUDE_PDOU
#define CONFIG_INCLUDE_PDOK
#define CONFIG_INCLUDE_NMT_MN
#define CONFIG_INCLUDE_SDOS
#define CONFIG_INCLUDE_SDOC
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_DLLU
#define CONFIG_INCLUDE_NMT_CN
#define CONFIG_INCLUDE_NMTK
#define CONFIG_INCLUDE_NMTU
#define CONFIG_INCLUDE_CFM

// =========================================================================
// EPL ethernet driver (Edrv) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
#define EDRV_FAST_TXFRAMES                      FALSE

// switch this define to TRUE if Edrv supports early receive interrupts
#define EDRV_EARLY_RX_INT                       FALSE

// enables setting of several port pins for benchmarking purposes
#define EDRV_BENCHMARK                          FALSE

// Call Tx handler (i.e. EplDllCbFrameTransmitted()) already if DMA has finished,
// otherwise call the Tx handler if frame was actually transmitted over ethernet.
#define EDRV_DMA_TX_HANDLER                     FALSE

// number of used ethernet controller
//#define EDRV_USED_ETH_CTRL                    1

// Disable deferred release of rx-buffers
#define DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS    FALSE

#define EDRV_USE_DIAGNOSTICS                    TRUE
#define EDRV_CYCLIC_USE_DIAGNOSTICS             FALSE


// =========================================================================
// Data Link Layer (DLL) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoC
#define EPL_DLL_PRES_READY_AFTER_SOC            FALSE

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoA
#define EPL_DLL_PRES_READY_AFTER_SOA            FALSE

// activate PResChaining support on MN
#define EPL_DLL_PRES_CHAINING_MN                TRUE


// =========================================================================
// OBD specific defines
// =========================================================================

// switch this define to TRUE if Epl should compare object range
// automaticly
#define CONFIG_OBD_CHECK_OBJECT_RANGE                  FALSE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
//#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM        FALSE
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM            TRUE

#define CONFIG_OBD_USE_LOAD_CONCISEDCF                 TRUE

#define CONFIG_OBD_DEF_CONCISEDCF_FILENAME             "/ata0a/mnobd.cdc"

#define CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART         TRUE

// =========================================================================
// Timer module specific defines
// =========================================================================

// if TRUE the high resolution timer module will be used
#define EPL_TIMER_USE_HIGHRES                   TRUE

#define EPL_CFM_CONFIGURE_CYCLE_LENGTH          TRUE

// =========================================================================
// SDO module specific defines
// =========================================================================

// increase the number of SDO channels, because we are master
#define SDO_MAX_CONNECTION_ASND                 100
#define MAX_SDO_SEQ_CON                         100
#define MAX_SDO_COM_CON                         100
#define SDO_MAX_CONNECTION_UDP                  50


#endif //_EPLCFG_H_



