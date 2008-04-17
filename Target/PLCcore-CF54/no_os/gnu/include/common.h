/*
 * File:		common.h
 * Purpose:		File to be included by all project files
 *
 * Notes:
 */

#ifndef _COMMON_H_
#define _COMMON_H_

/********************************************************************/

/*
 * Include the CPU header file
 */
#include "plccore.h"
//#include "../include/mcf548x.h"
#include "mcf548x/mcf548x_eport.h"
#include "mcf548x/mcf548x_intc.h"
//#include "mcf548x/mcf548x_gpio.h"
#include "mcf548x/mcf548x_psc.h"

/*
 * Include the board specific header file
 */

//#include "PLCcore-MCF548x.h"

/*
 * Include any toolchain specfic header files
 */


/*
 * Include common utilities
 */
#include "../include/assert.h"

/********************************************************************/

#endif /* _COMMON_H_ */
