################################################################################
#
# Build BOOT.bin Makefile
#
# Copyright (c) 2014, Kalycito Infotech Private Limited
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

VERSION = 0.1

ARM-OBJCOPY=arm-xilinx-eabi-objcopy
DATA2MEM=data2mem
BOOTGEN=bootgen
OPLKDRV_DAEMON=oplkdrv_daemon
REMOVE_SECTION=-R .local_memory -R .vectors.*
.PHONY: header
header:
	@echo ""
	@echo "================================================================================"
	@echo " Build Boot.bin for Sd card"
	@echo "================================================================================"
	@echo ""
	@echo " Copyright (c) 2014 B&R"
	@echo " Version $(VERSION)"
	@echo "================================================================================"
	@echo ""
	@echo "Write 'make all' to build boot.bin file"
	@echo ""
	@echo "Write 'make clean-bin' to delete the boot.bin "

.PHONY: all
all: clean-bin copy-files build-bin

####################################################
# D O W N L O A D
####################################################
.PHONY: clean-files
clean-bin:
	rm -f BOOT.bin

.PHONY: copy-files
copy-files:
	@echo Copying files
	cp ../../../microblaze/xilinx-z702/mn-dual-shmem-gpio/$(OPLKDRV_DAEMON).elf .
.PHONY: build-bin
build-bin:
	$(DATA2MEM) -bm system_bd.bmm -bt system.bit -bd $(OPLKDRV_DAEMON).elf tag pcp -o b download.bit
	$(ARM-OBJCOPY) -I elf32-little -O binary $(REMOVE_SECTION) $(OPLKDRV_DAEMON).elf $(OPLKDRV_DAEMON).bin
	$(BOOTGEN) -image bootimage.bif -o i BOOT.BIN -w on
