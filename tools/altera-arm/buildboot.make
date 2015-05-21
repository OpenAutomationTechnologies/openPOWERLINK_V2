################################################################################
#
# Build BOOT.bin Makefile
#
# Copyright (c) 2015, Kalycito Infotech Private Limited
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

.PHONY: header
header:
	@echo -e "\033[32m========================================================================="
	@echo "                RUNNING APP ON ARM FROM SD CARD                          "
	@echo "========================================================================="
	@echo "Application binary is placed in ${ARCH_INSTALL_POSTFIX}"
	@echo ""
	@echo "BOOT.bin: is the ARM openPOWERLINK application executable"
	@echo "preloader-mkimage.bin:        is the ARM preloader"
	@echo "------------------------------------------------------------------------"
	@echo "Prepare an SD card using the following command after connecting it:"
	@echo ""
	@echo ""
	@echo -e "\033[33m  # dd if=${SOCEDS_DEST_ROOT}/embeddedsw/socfpga/prebuilt_images/sd_card_linux_boot_image.img of=/dev/<sd_card_drive> bs=1M"
	@echo ""
	@echo "  \$$ alt-boot-disk-util.exe -p preloader-mkpimage.bin -a write -d <sd_card_path>"
	@echo ""
	@echo -e "  \$$ cp BOOT.bin <sd_card_path>"
	@echo ""
	@echo -e "  \$$ cp fpga.rbf <sd_card_path>\033[32m"
	@echo ""
	@echo "------------------------------------------------------------------------"
	@echo "To find the SD card drive path do the following:"
	@echo "1. Remove the sdcard"
	@echo "2. Type the following"
	@echo ""
	@echo -e "\033[33m    \$$ cat /proc/partitions\033[32m"
	@echo ""
	@echo "3. note down the devices listed"
	@echo "4. Connect the SD card"
	@echo "5. Type the following again"
	@echo ""
	@echo -e "\033[33m    \$$ cat /proc/partitions\033[32m"
	@echo ""
	@echo "6. note down the devices listed"
	@echo "The difference between the two lists is the device drive path for the SD card"
	@echo ""
	@echo -e "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\033[0m"

.PHONY: all
all: header
