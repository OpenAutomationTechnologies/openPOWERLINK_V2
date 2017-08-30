// file: boot_loader_epcs_bits.S
// asmsyntax=nios2
//
// Copyright 2009 Altera Corporation, San Jose, California, USA.
// All rights reserved.
//
// written by TPA, moved around by dvb, 2003-2004
// routines for accessing data out of EPCS serial
// flash device. This device is made of registers
// and you gotta talk to the registers to get
// your bytes.
//
// optimized by kds 2004.
//
// re-factored by jrk, 2006.01.26:
// - FPGA device-family-specific code moved to
//   separate sources.
// - This file contains routines common to the
//   boot-loader for the interface to the EPCS
//   Controller peripheral.
//
// added simulation hooks, 2009.08.17:
// - Passing EPCS_SIMULATION_TEST to the pre-processor
//   during build disables certain EPCS functionality
//   to allow for testing without a physical EPCS
//   device or simulation model. This is inteded to
//   be used to validate portions of the boot-loader
//   code that are device-dependent.
// - Passing EPCS_SIMULATION_TEST_FLASH_BASE=<base> to
//   the pre-processor may be used in conjunction with
//   EPCS_SIMULATION_TEST to direct the boot loader to look
//   at a particular base address for the flash payload
//   while under test.
//
// Updated by jkempa 2012.06.15:
// Refactoring new support for EPCQ 256 devices. The changes
// make the boot loader too big for Cyclone/Cyclone II. They won't
// support the new chip Johnny, sorry about that.

#include "boot_loader.h"

    .global sub_streaming_copy_epcs
    .global sub_epcs_hang_forever

// |
// | Let the code begin
// |

    .text

////////
// epcs_hang_forever
//
// We found nothing of interest. Hang -- but be good and close the
// EPCS device first. If someone comes along and resets the CPU to do
// something useful, this will allow proper EPCS access.
//
sub_epcs_hang_forever:
    nextpc  return_address_less_4
    br      sub_epcs_close
fp_hang:
    br      fp_hang


////////
// Streaming copy
//
//   Copies r_data_size bytes from r_flash_ptr to r_dest
//
//   Register usage:
//       argument:   r_data_size - number of bytes to copy
//       argument:   r_dest      - destination of the copy
//       implied:    r_flash_ptr - source address for the copy
//       temporary:  rf_temp
//       return-value : none
//
//   All args are smashed by this routine
//
//   Note: we don't keep the flash ptr up to date.  Instead
//           we just keep streaming from the EPCS device
//
sub_streaming_copy_epcs:
    // Fix-up return-address  (NOTE: LEAF)
    addi    return_address_less_4, return_address_less_4, 4

#ifndef EPCS_SIMULATION_TEST
    // for legibility
    #define r_dest_end r_data_size

    // convert the length to the ending address
    add     r_dest_end, r_data_size, r_dest
    subi    r_dest_end, r_dest_end, 1

    // Wait until controller is ready for a TX-char
epcs_copy_initial_wait:
    ldwio   rf_temp, EPCS_STATUS_OFFSET (r_epcs_base_address)
    andi    rf_temp, rf_temp, EPCS_STATUS_TRDY_MASK
    beq     rf_temp, r_zero, epcs_copy_initial_wait

    // prime the stream by sending the initial zero
    stwio   r_zero, EPCS_TXDATA_OFFSET (r_epcs_base_address)

    //
    // do {
    //   *r_dest++ = (char*)r_flash_ptr++)
    // while (r_dest <= r_dest_end);
    //
epcs_copy_loop:
    // Wait until an RX-character is available
    ldwio   rf_temp, EPCS_STATUS_OFFSET (r_epcs_base_address)
    andi    rf_temp, rf_temp, EPCS_STATUS_RRDY_MASK
    beq     rf_temp, r_zero, epcs_copy_loop

    // grab the RX-character, and immediately ask for another one
    //   no need to wait for TX ready, if RX is ready, then TX is too
    ldwio   rf_temp, EPCS_RXDATA_OFFSET (r_epcs_base_address)
    stwio   r_zero, EPCS_TXDATA_OFFSET (r_epcs_base_address)

    // store the character we retrieved, and update the destination ptr
    stbio   rf_temp, 0(r_dest)
    addi    r_dest, r_dest, 1

    // loop until the destination == the ending address
    bne     r_dest, r_dest_end, epcs_copy_loop

epcs_copy_last_wait:
    // Wait until an RX-character is available
    ldwio   rf_temp, EPCS_STATUS_OFFSET (r_epcs_base_address)
    andi    rf_temp, rf_temp, EPCS_STATUS_RRDY_MASK
    beq     rf_temp, r_zero, epcs_copy_last_wait

    // grab the last RX-character
    ldwio   rf_temp, EPCS_RXDATA_OFFSET (r_epcs_base_address)

    // store the last character
    stbio   rf_temp, 0(r_dest)

#else /* EPCS_SIMULATION_TEST */

    // for legibility
    #define r_dest_end_plus_one r_data_size

    // convert the length to the ending address + 1
    //   same number of instructions, but one less in the loop
    add     r_dest_end_plus_one, r_data_size, r_dest

    //
    // do {
    //   *r_dest++ = (char*)r_flash_ptr++)
    // while (r_dest != r_dest_end_plus_one);
    //
sim_test_copy_loop:
    ldbuio  rf_temp, 0(r_flash_ptr)
    addi    r_flash_ptr, r_flash_ptr, 1
    stbio   rf_temp, 0(r_dest)
    addi    r_dest, r_dest, 1

    // loop until the destination == 1 + the ending address
    bne     r_dest, r_dest_end_plus_one, sim_test_copy_loop
#endif /* EPCS_SIMULATION_TEST */

    // Return
    jmp     return_address_less_4   // Don't worry--we fixed it.

// end of file
