// file: boot_loader.h
// asmsyntax=nios2
//
// Copyright 2003-2004 Altera Corporation, San Jose, California, USA.
// All rights reserved.
//
// Register definitions for the boot loader code

// Symbolic definitions for how registers are used in this program
// program
#define r_zero                      r0
#define r_asm_tmp                   r1

#define r_flash_ptr                 r2
#define r_data_size                 r3
#define r_dest                      r4

#define r_halt_record               r5

#define r_read_int_return_value     r6
#define r_riff_count                r7
#define r_riff_return_address       r8
#define rf_temp                     r9

#define r_read_byte_return_value    r10

#define r_epcs_tx_value             r11

#define r_eopen_eclose_tmp          r12

#define r_findp_return_address      r13
#define r_findp_temp                r14
#define r_findp_pattern             r15
#define r_findp_count               r16

#define r_revbyte_mask              r17

#define r_epcs_base_address         r18

#define r_flush_counter             r19
#define r_my_temp                   r19 // Used to calc the right EPCS image offset

#define r_trie_count                r20

#define r_epcs_4_bytes_mode         r21

#define r_open_close_return_address r22

#define return_address_less_4       r23

// EPCS control/status register offsets
#define EPCS_RXDATA_OFFSET  0x00
#define EPCS_TXDATA_OFFSET  0x04
#define EPCS_STATUS_OFFSET  0x08
#define EPCS_CONTROL_OFFSET 0x0C

// EPCS Bit Masks
#define EPCS_STATUS_TMT_MASK  0x20
#define EPCS_STATUS_TRDY_MASK 0x40
#define EPCS_STATUS_RRDY_MASK 0x80

#define EPCS_CONTROL_SSO_MASK 0x400

// EPCS commands
#define EPCS_COMMAND_READ 0x03
#define EPCS_COMMAND_RDID 0x9F
#define EPCS_COMMAND_EN4B 0xB7
#define EPCS_COMMAND_EX4B 0xE9
#define EPCS_COMMAND_WREN 0x06

// Density ID
#define EPCS_256          0x19

// end of file
