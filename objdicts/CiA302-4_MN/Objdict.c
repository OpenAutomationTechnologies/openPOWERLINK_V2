
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
#include "obd.h"            // function prototypes of the EplOBD-Modul
#include "user/pdou.h"      // function prototype of OD callback function
#include "user/errhndu.h"   // function prototype of error handler od callback functions
#include "user/ctrlu.h"
#include "obdcfg.h"
#include "EplObjDef.h"
