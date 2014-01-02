#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx Examples
# Module        : download .elf file
# Contents      : connects to mb, download the elf file and starts execution
################################################################################

if { $argc != 2 } {
    #Exit on error
    puts "ERROR: Invalid download script parameters!"
    exit 1;
}

set enVerify [lindex $argv 1]

# Establish connection to debug module
connect mb mdm

# Read name of executeable from script arguments
set executeable "[lindex $argv 0].elf"
dow $executeable

if { $enVerify } {
    puts "INFO: Verify elf download. This can take a few minutes!"
    elf_verify
}

run

disconnect 0
exit;
