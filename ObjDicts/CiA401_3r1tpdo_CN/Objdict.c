
//
//===========================================================
// function name for initialisation of RAM for OD instance 
//
//===========================================================

#define EPL_OBD_INIT_RAM_NAME    EplObdInitRam

// new define is necessary
#define EPL_OBD_LINKED_INSTANCE     1
//
//===========================================================
// includes
//
//===========================================================

#include "Epl.h"            // function prototype of OD callback function
#include "EplObd.h"         // function prototypes of the EplOBD-Modul
#include "user/EplPdou.h"   // function prototype of OD callback function
#include "obdcfg.h" 
#include "EplObjDef.h"
