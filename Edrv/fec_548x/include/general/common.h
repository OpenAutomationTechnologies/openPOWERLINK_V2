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
#include "mcf5xxx.h"
#include "assert.h"


/* 
 * Include the board specific header file 
 
#if (defined(VERDI))
#include "src/include/verdi.h"
#elif (defined(M5485EVB))
#include "src/include/m5485evb.h"
#endif
*/
/* 
 * Include any toolchain specfic header files 
 
#if (defined(__MWERKS__))
#include "src/include/mwerks.h"
#elif (defined(__DCC__))
#include "src/include/diab.h" 
#elif (defined(__ghs__))
#include "src/include/ghs.h"
#endif
/*
/* 
 * Include common utilities
 */
//#include "src/include/assert.h"

/********************************************************************/

#define   INTC_ICRn(x) 	 	(*(vuint8 *)(void*)(MCF_MBAR+0x000740+((x)*0x001)))
#define   INTC_IMRH			*(vuint32*)(MCF_MBAR + 0x000708)
#define   INTC_IMRL			*(vuint32*)(MCF_MBAR + 0x00070C)
  	


#endif /* _COMMON_H_ */
