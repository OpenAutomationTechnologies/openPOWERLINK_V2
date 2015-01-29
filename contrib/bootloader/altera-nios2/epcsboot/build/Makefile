################################################################################
#
# Makefile for EPCS bootloader
#
# Note: This makefile is created by referencing on the bootloader provided by
#       Altera.
#
# Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

.NOTPARALLEL:
SHELL := /bin/bash

BUILD_DIR = .
SRC_DIR = ../src

OUT_FILE_NAME = epcsboot

SRC_FILES = $(SRC_DIR)/boot_loader.S \
			$(SRC_DIR)/boot_loader_epcs_bits.S \
			$(SRC_DIR)/boot_loader_epcs_bits_sii_siii_ciii.S

ELF_FILE = $(BUILD_DIR)/$(OUT_FILE_NAME).elf
HEX_FILE = $(BUILD_DIR)/$(OUT_FILE_NAME).hex
DUMP_FILE = $(BUILD_DIR)/$(OUT_FILE_NAME).dump

CODE_SPAN = 1024

OPTIONS = -DEPCS

ifeq ("$(CODE_BASE)", "")
	CODE_BASE=0x0
endif

OPTIONS += -Ttext=$(CODE_BASE)

ifneq ("$(EPCS_REGS_BASE)","")
	OPTIONS += -DEPCS_REGS_BASE=$(EPCS_REGS_BASE)
endif

ifneq ("$(REMOTE_UPDATE_BASE)", "")
	OPTIONS += -DREMOTE_UPDATE_BASE=$(REMOTE_UPDATE_BASE)
endif

all: $(HEX_FILE) $(DUMP_FILE)
	@echo CODE_BASE = $(CODE_BASE)
	@echo EPCS_REGS_BASE = $(EPCS_REGS_BASE)
	@echo REMOTE_UPDATE_BASE = $(REMOTE_UPDATE_BASE)

clean:
	@rm -rf $(HEX_FILE) $(ELF_FILE) $(DUMP_FILE)

help:
	@echo "$ make [COMMAND] [OPTIONS]"
	@echo "This compiles the EPCS flash controller bootloader."
	@echo "It generates an elf and hex file."
	@echo
	@echo "COMMAND:"
	@echo " all ................... Builds the bootloader and generates srec and hex files."
	@echo " clean ................. Cleans the generates files."
	@echo
	@echo "OPTIONS (OPTION=VALUE):"
	@echo " CODE_BASE ............. Specifies the base of the bootloader code."
	@echo "                         If not set, 0x0 is used."
	@echo " REMOTE_UPDATE_BASE .... Set remote update core base address."
	@echo "                         If not set, the bootloader will always boot from zero flash offset."
	@echo
	@echo "EXAMPLE:"
	@echo " Compile the bootloader for code base 0x400000."
	@echo " The remote update core base is 0x600000."
	@echo "  make all CODE_BASE=0x400000 REMOTE_UPDATE_BASE=0x600000."

$(ELF_FILE): $(SRC_FILES)
	@echo "INFO: Build $@"
	@nios2-elf-gcc -nostdlib $(SRC_FILES) $(OPTIONS) -gdwarf2 -Wa,-gdwarf2 -o $@

$(HEX_FILE): $(ELF_FILE)
	@echo "INFO: Generate $@"
	@elf2hex --input $< --output $@ --width=32 \
	--base=$(CODE_BASE) --end=$$(( $(CODE_BASE) + $(CODE_SPAN) - 1 )) --record=4 --lower


$(DUMP_FILE): $(ELF_FILE)
	@echo "INFO: Generate $@"
	@nios2-elf-objdump $< -D --source > $@
