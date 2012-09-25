#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx Examples
# Module        : build system
# Autor         : mairt
# Date          : 13.02.2012
# File          : Makefile
# contents      : Controls the build of the Xilinx examples
################################################################################

VERSION = 0.3

include makefile.settings

APP_DIR=${STACK_ROOT}/Examples/xilinx_microblaze/no_os/gnu/demo_cn_digitalio
OBJDICT_DIR=${STACK_ROOT}/ObjDicts/CiA401_CN
EDRV_DIR=${STACK_ROOT}/Edrv/openmac
TARGET_DIR=${STACK_ROOT}/Target/xilinx_microblaze/no_os/gnu/generic

IP_CORE_REPO=${STACK_ROOT}/Examples/xilinx_microblaze/ipcore

HW_SPEC=$(XPS_DIR)/SDK/SDK_Export/hw
BSP_PATH=$(APP_DIR)

OBJDIR=obj

##################################
#general compile defines
CC=mb-gcc
AR=mb-ar
CP=cp
RM=rm -rf
EXECUTABLE=directIO

LSSCRIPT=lscript_$(BOARD_NAME)_$(BUS_INTERFACE).ld
MSS_FILE=system_$(BOARD_NAME)_$(BUS_INTERFACE).mss
MACROS=-D${DBG_MODE} -DDEF_DEBUG_LVL=${DEF_DEBUG_LVL}
CFLAGS=-g3 -Wall -fmessage-length=0 $(PROC_FEATURES) ${MACROS}
CPPFLAGS=-MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
LDFLAGS=-Wl,-T -Wl,${LSSCRIPT} $(PROC_FEATURES) -Wl,-Map,${EXECUTABLE}.map

# Flash defines
FLASH_IMAGE_NAME= flash_image
BOOTLOADER_NAME= mem_bootloader
BOOT_BUILD_DIR= build
OBJCPYFLAGS=-O srec
BITSTREAM= download


##################################
# Include directory
INCLUDES=-I${APP_DIR}\
-I${EDRV_DIR}/include \
-I${STACK_ROOT}/Include \
-I${STACK_ROOT}/Include/user \
-I${STACK_ROOT}/Include/kernel \
-I${STACK_ROOT}/SharedBuff \
-I${STACK_ROOT}/ObjDicts \
-I${OBJDICT_DIR} \
-I${PROCESSOR_NAME}/include \
-I${TARGET_DIR}

##################################
# Source files
SRCFILES=${APP_DIR}/demo_main.c \
${OBJDICT_DIR}/Objdict.c \
${EDRV_DIR}/source/EdrvOpenMac.c \
${EDRV_DIR}/source/EplTimerSynck_OpenMac.c \
${EDRV_DIR}/source/EplTgtTimeStamp_OpenMac.c \
${EDRV_DIR}/source/omethlib.c \
${EDRV_DIR}/source/omethlibint.c \
${EDRV_DIR}/source/omethlib_phycfg_generic.c \
${STACK_ROOT}/EplStack/EplDllk.c \
${STACK_ROOT}/EplStack/EplDllkCal.c \
${STACK_ROOT}/EplStack/EplDlluCal.c \
${STACK_ROOT}/EplStack/EplLedu.c \
${STACK_ROOT}/EplStack/EplEventk.c \
${STACK_ROOT}/EplStack/EplEventu.c \
${STACK_ROOT}/EplStack/EplNmtk.c \
${STACK_ROOT}/EplStack/EplNmtu.c \
${STACK_ROOT}/EplStack/EplNmtCnu.c \
${STACK_ROOT}/EplStack/EplPdok.c \
${STACK_ROOT}/EplStack/EplPdokCal.c \
${STACK_ROOT}/EplStack/EplPdou.c \
${STACK_ROOT}/EplStack/EplPdouCal.c \
${STACK_ROOT}/EplStack/EplObd.c \
${STACK_ROOT}/EplStack/EplSdoComu.c \
${STACK_ROOT}/EplStack/EplSdoAsySequ.c \
${STACK_ROOT}/EplStack/EplSdoAsndu.c \
${STACK_ROOT}/EplStack/EplTimeruGeneric.c \
${STACK_ROOT}/EplStack/EplErrorHandlerk.c \
${STACK_ROOT}/EplStack/EplApiGeneric.c \
${TARGET_DIR}/ShbTarget_Microblaze.c \
${TARGET_DIR}/xilinx_irq.c \
${TARGET_DIR}/xilinx_usleep.c \
${STACK_ROOT}/SharedBuff/SharedBuff.c \
${STACK_ROOT}/SharedBuff/ShbIpc-NoOS.c \
${APP_DIR}/systemComponents.c

LIBRARIES = libxil.a

VPATH=$(sort $(patsubst " ", :, $(dir $(SRCFILES))))


###################################
# set Optimization level makro
ifeq ($(OPT_LEVEL),-O0)
	MACROS+= -DXIL_NO_OPT_LEVEL
endif
ifeq ($(OPT_LEVEL),-O1)
	MACROS+= -DXIL_OPT_LEVEL_1
endif
ifeq ($(OPT_LEVEL),-O2)
	MACROS+= -DXIL_OPT_LEVEL_2
endif
ifeq ($(OPT_LEVEL),-O3)
	MACROS+= -DXIL_OPT_LEVEL_3
endif
ifeq ($(OPT_LEVEL),-Os)
	MACROS+= -DXIL_OPT_LEVEL_SIZE
endif

###################################
# set endian according to bus interface
ifeq ($(BUS_INTERFACE),axi)
	PROC_FEATURES+= -mlittle-endian
	SRCFILES+= ${STACK_ROOT}/EplStack/amiarm.c
else
	PROC_FEATURES+= -mbig-endian
	SRCFILES+= ${STACK_ROOT}/EplStack/amibe.c
endif

###################################
# libraries
LIBSPATH=-L$(BSP_PATH)/${PROCESSOR_NAME}/lib

###################################
# objects
OBJ = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.o, $(SRCFILES))))
OBJD = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.d, $(SRCFILES))))

OUTPUT_OPTION = -o $@

CFLAGS += ${INCLUDES}

.PHONY:	header
header:
	@echo ""
	@echo "================================================================================"
	@echo " Direct IO - Build System (Xilinx)"
	@echo "================================================================================"
	@echo ""
	@echo " Copyright (c) 2012 B&R"
	@echo " Version $(VERSION)"
	@echo "================================================================================"
	@echo ""
	@echo "Write 'make all' to generate the bitstream and Direct I/O software example"
	@echo ""
	@echo "Write 'make application' to generate the Direct I/O software example"
	@echo "Write 'make bitstream' to generate the bitstream"
	@echo ""
	@echo "Write 'make download-bits' to download the bitstream to the target"
	@echo "Write 'make download-elf' to download the .elf file to the target"
	@echo ""
	@echo "Write 'make clean_sw' to clean the application"
	@echo "Write 'make clean_hw' to clean the bitstream"
	@echo "Write 'make clean_all' to clean the all generated files"
	@echo ""
	@echo "Change 'makefile.setting' to configure the build system"

.PHONY: all
all: bitstream application

####################################################
# A P P L I C A T I O N
####################################################
.PHONY: application
application: builddir ${EXECUTABLE}.elf ${EXECUTABLE}.size ${EXECUTABLE}.elfcheck ${EXECUTABLE}.srec

$(BSP_PATH)/$(PROCESSOR_NAME) : $(HW_SPEC)/system.xml $(MSS_FILE)
	@echo Invoking: libgen
	libgen -hw $(HW_SPEC)/system.xml -lp $(IP_CORE_REPO) -pe $(PROCESSOR_NAME) -od $(BSP_PATH) -log libgen.log $(MSS_FILE)
	@echo Finished building: library
	@echo ' '

.PHONY: builddir
builddir:
	$(shell mkdir $(OBJDIR))

$(OBJDIR)/%.o: %.c $(BSP_PATH)/$(PROCESSOR_NAME)
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c ${OPT_LEVEL} $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

# extra rule for usleep to set opt level to zero
$(OBJDIR)/xilinx_usleep.o: xilinx_usleep.c xilinx_usleep.h $(BSP_PATH)/$(PROCESSOR_NAME)
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c -O0 $(CFLAGS) $(CPPFLAGS) $< ${OUTPUT_OPTION}
	@echo Finished building: $<
	@echo ' '

${EXECUTABLE}.elf: $(BSP_PATH)/$(PROCESSOR_NAME) ${OBJ} ${LSSCRIPT}
	@echo Building target: $@
	@echo Invoking: MicroBlaze gcc linker
	$(CC) $(LDFLAGS) $(LIBSPATH) ${OBJ} -o ${EXECUTABLE}.elf
	@echo Finished building target: $@
	@echo ' '

${EXECUTABLE}.size: ${EXECUTABLE}.elf
	@echo Invoking: MicroBlaze Print Size
	mb-size ${EXECUTABLE}.elf  |tee "${EXECUTABLE}.size"
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.elfcheck: ${EXECUTABLE}.elf
	@echo Invoking: Xilinx ELF Check
	elfcheck ${EXECUTABLE}.elf -hw ${HW_SPEC}/system.xml -pe ${PROCESSOR_NAME}  |tee "${EXECUTABLE}.elfcheck"
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.srec: ${EXECUTABLE}.elf
	@echo Invoking: mb-objcopy
	mb-objcopy $(OBJCPYFLAGS) ${EXECUTABLE}.elf ${EXECUTABLE}.srec
	@echo Finished building: $@
	@echo ' '

${EXECUTABLE}.mem: ${EXECUTABLE}.elf
	@echo Invoking: Data2MEM
	Data2MEM -bd ${EXECUTABLE}.elf -d -o m ${EXECUTABLE}.mem
	@echo Finished building: $@
	@echo ' '

####################################################
# C L E A N
####################################################
.PHONY: clean
clean: clean_sw

.PHONY: clean_all
clean_all: clean_sw clean_hw

.PHONY: clean_sw
clean_sw:
	-@echo "Cleaning software parts..."
	@touch $(EXECUTABLE)
	-${RM} -rf $(EXECUTABLE).* $(OBJ) $(OBJD) $(PROCESSOR_NAME)
	-@echo ' '

.PHONY: clean_hw
clean_hw:
	-@echo "Cleaning bitstream parts..."
	$(MAKE) -C $(XPS_DIR) -f system.make clean
	-${RM} -rf $(XPS_DIR)/system.make
	-@echo ' '


####################################################
# H A R D W A R E
####################################################

$(XPS_DIR)/system.make :
	$(MAKE) -C $(XPS_DIR) bitstream

$(XPS_DIR)/implementation/system.bit :
	$(MAKE) -C $(XPS_DIR) -f system.make bits

$(HW_SPEC)/system.xml :
	$(MAKE) -C $(XPS_DIR) -f system.make exporttosdk

.PHONY: bitstream
bitstream: $(XPS_DIR)/system.make $(XPS_DIR)/implementation/system.bit $(HW_SPEC)/system.xml


####################################################
# D O W N L O A D
####################################################
.PHONY: download-bits
download-bits: $(XPS_DIR)/system.make
	$(MAKE) -C $(XPS_DIR) -f system.make download

.PHONY: download-elf
download-elf: application
	xmd -hw $(HW_SPEC)/system.xml -tcl download-elf.tcl

-include $(OBJD)
