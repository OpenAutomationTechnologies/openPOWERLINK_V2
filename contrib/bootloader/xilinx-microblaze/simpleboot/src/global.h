/*
 * Copyright (c) 2010 Avnet.  All rights reserved.
 *
 * Avnet (HK)
 * AVNET IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, AVNET IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION
 * AVNET EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "xparameters.h"

/* debug settings */
#define DEBUG                            0      // 1 - general debug message
#define DEBUG_SPI                        0      // 1 - debug spi driver
#define DEBUG_PRINT_FLASH_DATA           0      // 1 - debug readback spi flash data

/**************************************************/
/* Automatic settings */
/**************************************************/

/* bootloader mode */
#ifdef XPAR_SPI_FLASH_BASEADDR
  #define SPI_FLASH                        1      // 1 - SPI flash; 0 - parallel flash
#else
  #define SPI_FLASH                        0      // 1 - SPI flash; 0 - parallel flash
#endif //XPAR_SPI_FLASH_BASEADDR

/* Enable/Disable byte swapping */
#if XPAR_MICROBLAZE_ENDIANNESS == 0           /* MICROBLAZE is big endian */
  #define SWAP_HILO_BYTE                 0    // 1 - for 16bit access, this can swap hi-lo bytes
#elif XPAR_MICROBLAZE_ENDIANNESS == 1         /* MICROBLAZE is little endian */
  #define SWAP_HILO_BYTE                 1    // 1 - for 16bit access, this can swap hi-lo bytes
#else
  #error "Microblaze endianess information is not available!"
#endif // XPAR_MICROBLAZE_ENDIANNESS == 0

#if SPI_FLASH == 1
  #define BIT_SWAP_ON                  0      // 1 - swap bit order
#elif SPI_FLASH == 0
  #define BIT_SWAP_ON                  0      // 1 - swap bit order
#else
  #error "Wrong Flash mode provided! Only 1 (SPI Flash) and 0 (Parallel Flash) is possible!"
#endif // SPI_FLASH == 1


#endif

