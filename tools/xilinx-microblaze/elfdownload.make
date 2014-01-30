################################################################################
#
# Download program application Makefile
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

####################################################
# U S E R   O P T I O N S
####################################################
VERIFY_ELF=false

####################################################
# S E T T I N G S
####################################################
XMD=xmd
XMD_SCRIPT=xmd-downloadelf.tcl
IMPACT=impact
IMPACT_SCRIPT=download.cmd

ELF_NAME= $(wildcard *.elf)

.PHONY: header
header:
	@echo ""
	@echo "================================================================================"
	@echo " Download the bitstream and executeable to target!"
	@echo "================================================================================"
	@echo ""
	@echo " Copyright (c) 2014 B&R"
	@echo " Version $(VERSION)"
	@echo "================================================================================"
	@echo ""
	@echo "Write 'make all' to download the bitstream and the .elf file"
	@echo ""
	@echo "Write 'make download-bits' to download the bitstream to the target"
	@echo "Write 'make download-elf' to download the .elf file to the target"

.PHONY: all
all: download-bits download-elf

####################################################
# D O W N L O A D
####################################################
.PHONY: download-bits
download-bits:
	$(IMPACT) -batch $(IMPACT_SCRIPT)

.PHONY: download-elf
download-elf:
	$(XMD) -hw system.xml -tcl $(XMD_SCRIPT) $(ELF_NAME) $(VERIFY_ELF)
