// boot_loader.S
// asmsyntax=nios2
//
// Copyright 2003-2004 Altera Corporation, San Jose, California, USA.
// All rights reserved.
//
// This program is a boot-loader, designed to start a Nios II system
// from flash.  It copies a payload into RAM (presumably a program),
// then jumps-into the copied code.
//
// This is not a very complex job, but there is one requirement
// that makes this program tricky:
//
//   (*) We want to make the exact-same binary code run on any system,
//       at any (cache-line-aligned) address, and still work properly.
//
// Thus, this program must be position-independent, use no
// memory-variables, and derive all payload-specific data from the
// payload itself.
//
// This same program really has two lives:
//      1) A CFI flash-copier.
//      2) An EPCS flash-copier.
//
// A CFI flash-copier is pretty easy, because reading data from CFI
// flash is as simple as a "ldbuio" instruction.  But reading data
// from EPCS flash is a big deal--because it's serial.  We have to
// reproduce a subset of the EPCS access-routines right here in
// position-independent assembly-language, whilst using no memory
// variables.  And, indeed, we have done just that.
//
// In either case, the payload is identical (the same ordered
// sequence of bytes), and is interpreted by the boot-copier in
// the same way.  The only two differences between CFI and
// EPCS are:
//
//      1) The method used to READ bytes from flash.
//      2) The method used to FIND the first byte in the payload.
//
// You control which kind of boot-copier you get using the
// pre-processor symbol "EPCS".
//
// **** WHAT IT DOES
//
// EPCS or otherwise, this program executes a simple set of
// unconditional actions.  It copies its "payload" from flash into
// RAM, then jumps to it.  Presumably, the payload contains a program
// and its initialized data.  The payload is created by a
// utility at software-compile time (elf2flash).
//
// The elf2flash utility and this program have to agree on the
// location and structure of the payload, or else this will be a short trip.
// The *payload* is a sequence of bytes stored in flash-memory:
//
//    * For CFI flash, the first byte of the payload immediately
//      follows the last instruction in this boot-copier.  Thus,
//      the first byte of the payload is at the label
//      "end_of_boot_copier:"
//
//    * For EPCS flash, the first byte of the payload follows the last
//      byte of the device-configuration.  This is obnoxious, because
//      the device configuration is of indeterminate length.  The only
//      way to know where the device-configuration ends is to
//      read the configuration-header and extract the length.  This
//      boot-copier does this to find the payload.  The implicit
//      assumption is that there is, in fact, Cyclone device
//      configuration data stored in the EPCS flash, starting
//      at offset zero.

// |
// | dvb2004: Each "program record" looks like so:
// |          4 bytes Length L
// |          4 bytes Address A
// |
// | Generally, the next L bytes are then shoveled out
// | of flash (cfi or epcs) and stashed at authentic
// | RAM address A.
// |
// | If L is zero, then A is an address to JUMP to for your
// | application to execute.
// |
// | If L is 0xffffFFFF, then we ignore A and halt. This lets
// | an erased flash memory behave safely, and allows an EPCS
// | to contain a sof file but no program (we'll write the four
// | bytes of F when we program the sof).
// |
//
// optimized by kds 2004.
// updated in prep for the new MMU addressing scheme, by kds 2004.
//
////////////////////////////////

//
// Conditionally-define these two "funcion" names.  One way, you'll
// get a simple CFI boot-loader.  The other way, you'll get a complex
// EPCS boot-loader

#ifdef EPCS
 #define FIND_PAYLOAD   sub_find_payload_epcs
 #define READ_INT       sub_read_int_from_flash_epcs
 #define STREAMING_COPY sub_streaming_copy_epcs
 #define CLOSE_DEVICE   sub_epcs_close
#else
 #define FIND_PAYLOAD   sub_find_payload_cfi
 #define READ_INT       sub_read_int_from_flash_cfi
 #define STREAMING_COPY sub_streaming_copy_cfi
#endif

#include "boot_loader.h"

    // |
    // | all aliases of things the linker or the compiler
    // | or Tim Allen might expect to find at offset zero
    // | of some code. --dvb
    // |

    .global reset
    .global _start
    .global main

    .global end_of_boot_copier

reset:
_start:
main:
    // Clear the CPU's status-register, thereby disabling interrupts.
    // This is redundant after a "real" hardware-reset operation, but
    // people who deliberately jump-to-reset may derive some benefit from
    // this.  And, if not, at least it doesn't hurt anyone.
    //
    wrctl   status, r_zero

    ////////
    // The first thing we want to do is flush the instruction cache.
    //
    // In Nios II version 1.0, the maximum allowed cache-size is 64KBytes.
    //
    //    NOTE: If Nios II ever supports more than 64KByte caches,
    //          someone will need to change this code.
    //

    movhi   r_flush_counter,%hi(0x10000)
cache_loop:
    initi   r_flush_counter
    // don't flush the data cache, the boot copier doesn't access data.

    addi    r_flush_counter, r_flush_counter,-32
    bne     r_flush_counter, r_zero, cache_loop

    // then flush the pipeline
    flushp

    // r_flash_ptr = find_payload();
    //   note: if we need to save a couple of bytes, this would be a
    //          good routine to in-line.
    nextpc  return_address_less_4
    br      FIND_PAYLOAD

    ////////
    // Copy-Job.
    //
    // At the start of the loop, r_flash_ptr contains the address of the next
    // Program-Record to process.
    //
    // 1) Read the length-word (4-bytes) of the Program-Record (r_data_size)
    //    (if r_zero, exit loop).
    //
    // 2) Read the destination-address of this record (r_dest)
    //
    // 3) Inner-loop:
    //       Copy r_data_size bytes, one byte at a time: *r_dest++ = *r_flash_ptr++
    //

    // load the register we use to detect a halt (or an erased flash) with 0xFFFFFFFF
    subi    r_halt_record, r_zero, 1

per_record_loop:

    // r_data_size = READ_INT(r_flash_ptr++)
    nextpc  return_address_less_4
    br      READ_INT
    mov     r_data_size, r_read_int_return_value

    // r_dest = READ_INT(r_flash_ptr++)
    nextpc  return_address_less_4
    br      READ_INT
    mov     r_dest, r_read_int_return_value

    ////
    // Test to see if r_data_size (r_data_size) is r_zero.
    // When it is, we go run the program.
    //
    beq     r_data_size, r_zero, last_program_record


    //   ------------------------------------------
    // | A record length of -1 (0xffffFFFF) is
    // | is a HALT record.
    // |

halt_record_forever:
    beq     r_data_size, r_halt_record, halt_record_forever

    // |
    //   ------------------------------------------

    // use the streaming copy routines to move the data
    //   note: if we need to save a couple of bytes, this would be a
    //          good routine to in-line.
    nextpc  return_address_less_4
    br      STREAMING_COPY

    // When you get to here, you're done with the current record.
    // And, you know that it wasn't the last one (because it's
    // length-field wasn't zero--we checked.  So, that can only mean
    // one thing.  Time for the next record:
    br      per_record_loop

last_program_record:
    // The last Program-Record is the jump-record.  The
    // r_dest is the entry-point of the
    // program.  This is easy as cheese.
    //
    // People seem to like to "return" from their main-program, and then
    // they expet someting reasonable to happen.  Weird.


    // 2005.03.03 -- SPR 169431
    // Close the EPCS device properly before terminating the
    // boot-loader. Failing to perform this step can cause a HAL open()
    // call to open the EPCS device to fail unless a large (multi-second)
    // delay has passed.
#ifdef EPCS
    nextpc  return_address_less_4
    br      CLOSE_DEVICE
#endif

    callr   r_dest

afterlife:        // So...this is where programs go when they die.
    br      afterlife

    .end

// end of file
