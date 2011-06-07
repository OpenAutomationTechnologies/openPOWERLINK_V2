/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      EPL Stack

  Description:  configuration file

  -------------------------------------------------------------------------

                $RCSfile: EplCfg.h,v $

                $Author: D.Krueger $

                $Revision: 1.2 $  $Date: 2008-04-08 12:10:45 $

                $State: Exp $

                Build Environment:
                    ...

  -------------------------------------------------------------------------

  Revision History: 

  2006/06/06    k.t.: Start of Implementation

****************************************************************************/

#ifndef _EPLCFG_H_
#define _EPLCFG_H_




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

// This defines the target hardware. Here is encoded wich CPU and wich external
// peripherals are connected. For possible values refere to target.h. If
// necessary value is not available EPL stack has to
// be adapted and tested.
#define TARGET_HARDWARE                 TGTHW_PC_WRAPP

// Default defug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#define EPL_DEF_DEBUG_LVL           0xE0800000L
//   EPL_DBGLVL_OBD         =   0x00000004L
// * EPL_DBGLVL_ASSERT      =   0x20000000L
// * EPL_DBGLVL_ERROR       =   0x40000000L
// * EPL_DBGLVL_ALWAYS      =   0x80000000L

// EPL_MODULE_INTEGRATION defines all modules which are included in
// EPL application. Please add or delete modules for your application.
#define EPL_MODULE_INTEGRATION          (0x0000059L)//(0x00000050L)
// EPL_MODULE_OBDK        0x00000001L // OBD kernelspace module
// EPL_MODULE_PDOK        0x00000002L // PDO kernelspace module
// EPL_MODULE_NMT_MN      0x00000004L // NMT MN module
// EPL_MODULE_SDOS        0x00000008L // SDO Server module
// EPL_MODULE_SDOC        0x00000010L // SDO Client module
// EPL_MODULE_SDO_ASND    0x00000020L // SDO over Asnd module
// EPL_MODULE_SDO_UDP     0x00000040L // SDO over UDP module
// EPL_MODULE_SDO_PDO     0x00000080L // SDO in PDO module
// EPL_MODULE_NMT_CN      0x00000100L // NMT CN module
// EPL_MODULE_NMTU        0x00000200L // NMT userspace module  
// EPL_MODULE_NMTK        0x00000400L // NMT kernelspace module  
// EPL_MODULE_DLLK        0x00000800L // DLL kernelspace module
// EPL_MODULE_DLLU        0x00001000L // DLL userspace module
// EPL_MODULE_OBDU        0x00002000L // OBD userspace module

// =========================================================================
// OBD specific defines
// =========================================================================

// switch this define to TRUE if Epl should compare object range
// automaticly
//#define EPL_OBD_CHECK_OBJECT_RANGE          FALSE
#define EPL_OBD_CHECK_OBJECT_RANGE          TRUE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
//#define EPL_OBD_USE_STRING_DOMAIN_IN_RAM    FALSE
#define EPL_OBD_USE_STRING_DOMAIN_IN_RAM    TRUE

#define EPL_OBD_USE_VARIABLE_SUBINDEX_TAB TRUE

// if TRUE it uses the Obd module implementation of EPL kernel also in EPL user
#define EPL_OBD_USE_KERNEL                  TRUE


#endif //_EPLCFG_H_



