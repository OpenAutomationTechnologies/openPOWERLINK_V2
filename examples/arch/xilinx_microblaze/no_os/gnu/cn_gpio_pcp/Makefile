#  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#      A-5142 Eggelsberg, B&R Strasse 1
#      www.br-automation.com
#
# Project       : POWERLINK Xilinx examples
# Module        : Build system
# File          : Makefile
# contents      : Manages the build of the Xilinx examples
################################################################################

VERSION = 0.4

include makefile.settings

APP_DIR_ARCH=$(STACK_ROOT)/examples/arch/xilinx_microblaze/no_os/gnu/cn_gpio_pcp
APP_DIR_GEN=$(STACK_ROOT)/examples/demo_cn_embedded
APP_DIR_COM=$(STACK_ROOT)/examples/common

OBJDICT_DIR=$(STACK_ROOT)/objdicts/cia401_cn

LIBS_DIR=$(STACK_ROOT)/libs
KERNEL_DIR=$(STACK_ROOT)/stack/src/kernel
USER_DIR=$(STACK_ROOT)/stack/src/user
COMMON_DIR=$(STACK_ROOT)/stack/src/common
TARGET_DIR=$(STACK_ROOT)/stack/src/arch/xilinx_microblaze

IP_CORE_REPO=$(STACK_ROOT)/fpga/ipcore/xilinx

HW_SPEC=$(XPS_DIR)/SDK/SDK_Export/hw
BSP_PATH=$(APP_DIR_ARCH)

OBJDIR=obj

##################################
#general compile defines
CC=mb-gcc
AR=mb-ar
CP=cp
RM=rm -rf
PERL=xilperl
EXECUTABLE=demo

LSSCRIPT=lscript_$(BOARD_NAME)_$(BUS_INTERFACE).ld
MSS_FILE=system_$(BOARD_NAME)_$(BUS_INTERFACE).mss
MACROS=-D$(DBG_MODE) -DDEF_DEBUG_LVL=$(DEF_DEBUG_LVL)
CFLAGS=$(DEBUG_FLAG) -Wall -fmessage-length=0 $(PROC_FEATURES) $(MACROS)
CPPFLAGS=-MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
LDFLAGS=-Wl,-T -Wl,$(LSSCRIPT) $(PROC_FEATURES) -Wl,-Map,$(EXECUTABLE).map
UBLAZE_SLEEP= usleep

##################################
# Include directory
INCLUDES=-I$(STACK_ROOT)/include \
-I$(LIBS_DIR)/omethlib \
-I$(LIBS_DIR)/circbuf \
-I$(STACK_ROOT)/objdicts/cia401_cn \
-I$(STACK_ROOT)/stack/include/target/openmac \
-I$(STACK_ROOT)/stack/src \
-I$(STACK_ROOT)/stack/src/arch/xilinx_microblaze \
-I$(PROCESSOR_NAME)/include \
-I$(APP_DIR_ARCH) \
-I$(APP_DIR_GEN) \
-I$(APP_DIR_COM)/gpio \
-I$(APP_DIR_COM)/lcd

##################################
# Source files
SRCFILES=$(APP_DIR_GEN)/main.c \
$(APP_DIR_GEN)/event.c \
$(APP_DIR_GEN)/app.c \
$(APP_DIR_COM)/lcd/lcd.c \
$(APP_DIR_COM)/lcd/lcdl-null.c \
$(APP_DIR_COM)/gpio/gpio-microblaze.c \
$(LIBS_DIR)/omethlib/omethlib.c \
$(LIBS_DIR)/omethlib/omethlibint.c \
$(LIBS_DIR)/omethlib/omethlib_phycfg_generic.c \
$(LIBS_DIR)/circbuf/circbuffer.c \
$(LIBS_DIR)/circbuf/circbuf-noos.c \
$(LIBS_DIR)/trace/trace-printf.c \
$(COMMON_DIR)/debug.c \
$(COMMON_DIR)/errstring.c \
$(COMMON_DIR)/dll/dllcal-direct.c \
$(COMMON_DIR)/event/event.c \
$(KERNEL_DIR)/ctrl/ctrlk.c \
$(KERNEL_DIR)/ctrl/ctrlkcal-direct.c \
$(KERNEL_DIR)/dll/dllk.c \
$(KERNEL_DIR)/dll/dllkfilter.c \
$(KERNEL_DIR)/dll/dllkstatemachine.c \
$(KERNEL_DIR)/dll/dllkframe.c \
$(KERNEL_DIR)/dll/dllkevent.c \
$(KERNEL_DIR)/dll/dllkcal.c \
$(KERNEL_DIR)/dll/dllkcal-circbuf.c \
$(KERNEL_DIR)/edrv/edrv-openmac.c \
$(KERNEL_DIR)/errhnd/errhndk.c \
$(KERNEL_DIR)/errhnd/errhndkcal-local.c \
$(KERNEL_DIR)/event/eventk.c \
$(KERNEL_DIR)/event/eventkcalintf-circbuf.c \
$(KERNEL_DIR)/event/eventkcal-nooscircbuf.c \
$(KERNEL_DIR)/hrtimer/hrtimersync-openmac.c \
$(KERNEL_DIR)/nmt/nmtk.c \
$(KERNEL_DIR)/pdo/pdok.c \
$(KERNEL_DIR)/pdo/pdokcal.c \
$(KERNEL_DIR)/pdo/pdokcalmem-local.c \
$(KERNEL_DIR)/pdo/pdokcalsync-null.c \
$(KERNEL_DIR)/pdo/pdokcal-triplebufshm.c \
$(KERNEL_DIR)/timestamp/timestamp-openmac.c \
$(USER_DIR)/timer/timer-generic.c \
$(USER_DIR)/api/generic.c \
$(USER_DIR)/api/processimage.c \
$(USER_DIR)/ctrl/ctrlu.c \
$(USER_DIR)/ctrl/ctrlucal-direct.c \
$(USER_DIR)/dll/dllucal.c \
$(USER_DIR)/dll/dllucal-circbuf.c \
$(USER_DIR)/errhnd/errhndu.c \
$(USER_DIR)/errhnd/errhnducal-local.c \
$(USER_DIR)/event/eventu.c \
$(USER_DIR)/event/eventucalintf-circbuf.c \
$(USER_DIR)/event/eventucal-nooscircbuf.c \
$(USER_DIR)/nmt/nmtu.c \
$(USER_DIR)/nmt/nmtcnu.c \
$(USER_DIR)/obd/obd.c \
$(USER_DIR)/obd/obdcreate.c \
$(USER_DIR)/pdo/pdou.c \
$(USER_DIR)/pdo/pdoucal.c \
$(USER_DIR)/pdo/pdoucalmem-local.c \
$(USER_DIR)/pdo/pdoucalsync-null.c \
$(USER_DIR)/pdo/pdoucal-triplebufshm.c \
$(USER_DIR)/sdo/sdo-asndu.c \
$(USER_DIR)/sdo/sdo-sequ.c \
$(USER_DIR)/sdo/sdo-comu.c \
$(USER_DIR)/ledu.c \
$(TARGET_DIR)/systemtimer.c \
$(TARGET_DIR)/usleep.c \
$(TARGET_DIR)/target-microblaze.c \
$(TARGET_DIR)/lock-localnoos.c

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
	SRCFILES+= $(STACK_ROOT)/libs/ami/amiarm.c
else
	PROC_FEATURES+= -mbig-endian
	SRCFILES+= $(STACK_ROOT)/libs/ami/amibe.c
endif

###################################
# libraries
LIBSPATH=$(BSP_PATH)/$(PROCESSOR_NAME)/lib

###################################
# objects
OBJ = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.o, $(SRCFILES))))
OBJD = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.d, $(SRCFILES))))

OUTPUT_OPTION = -o $@

CFLAGS += $(INCLUDES)

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
	@echo "Write 'make bsp' to generate the board support package"
	@echo "Write 'make app' to generate the Direct I/O software example"
	@echo "Write 'make bitstream' to generate the bitstream"
	@echo ""
	@echo "Write 'make download-all' to download the bitstream and the .elf file to the target"
	@echo "Write 'make download-bits' to download the bitstream to the target"
	@echo "Write 'make download-elf' to download the .elf file to the target"
	@echo ""
	@echo "Write 'make clean-bsp' to clean the board support package"
	@echo "Write 'make clean-app' to clean the application"
	@echo "Write 'make clean-sw' to clean the whole software"
	@echo ""
	@echo "Write 'make clean-hw' to clean the bitstream"
	@echo "Write 'make clean-all' to clean the all generated files"
	@echo ""
	@echo "Change 'makefile.setting' to configure the build system"

.PHONY: all
all: bitstream app

####################################################
# A P P L I C A T I O N
####################################################
.PHONY: app
app: builddir bsp $(EXECUTABLE).elf $(EXECUTABLE).size $(EXECUTABLE).elfcheck

bsp: $(LIBSPATH)/libxil.a

$(LIBSPATH)/libxil.a: $(HW_SPEC)/system.xml $(MSS_FILE)
	@echo Invoking: libgen
	libgen -hw $(HW_SPEC)/system.xml -lp $(IP_CORE_REPO) -pe $(PROCESSOR_NAME) -od $(BSP_PATH) -log libgen.log $(MSS_FILE)
	@echo Finished building: board support package
	@echo ' '

.PHONY: builddir
builddir:
	$(shell mkdir $(OBJDIR))

$(OBJDIR)/%.o: %.c $(LIBSPATH)/libxil.a
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c $(OPT_LEVEL) $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

# extra rule for usleep to set opt level to zero
$(OBJDIR)/$(UBLAZE_SLEEP).o: $(UBLAZE_SLEEP).c $(UBLAZE_SLEEP).h $(LIBSPATH)/libxil.a
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c -O0 $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

$(EXECUTABLE).elf: $(OBJ) $(LSSCRIPT) $(LIBSPATH)/libxil.a
	@echo Building target: $@
	@echo Invoking: MicroBlaze gcc linker
	$(CC) $(LDFLAGS) -L$(LIBSPATH) $(OBJ) -o $(EXECUTABLE).elf
	@echo Finished building target: $@
	@echo ' '

$(EXECUTABLE).size: $(EXECUTABLE).elf
	@echo Invoking: MicroBlaze Print Size
	mb-size $(EXECUTABLE).elf  |tee "$(EXECUTABLE).size"
	@echo Finished building: $@
	@echo ' '

$(EXECUTABLE).elfcheck: $(EXECUTABLE).elf
	@echo Invoking: Xilinx ELF Check
	elfcheck $(EXECUTABLE).elf -hw $(HW_SPEC)/system.xml -pe $(PROCESSOR_NAME)  |tee "$(EXECUTABLE).elfcheck"
	@echo Finished building: $@
	@echo ' '

####################################################
# C L E A N
####################################################
.PHONY: clean
clean: clean-sw

.PHONY: clean-all
clean-all: clean-sw clean-hw

.PHONY: clean-bsp
clean-bsp:
	-@echo "Cleaning board support package..."
	-$(RM) $(PROCESSOR_NAME)
	-@echo ' '

.PHONY: clean-app
clean-app:
	-@echo "Cleaning software parts..."
	@touch $(EXECUTABLE)
	-$(RM) $(EXECUTABLE).* $(OBJ) $(OBJD)
	-@echo ' '

.PHONY: clean-sw
clean-sw: clean-bsp clean-app

.PHONY: clean-hw
clean-hw: $(XPS_DIR)/system.make
	-@echo "Cleaning bitstream parts..."
	$(MAKE) -C $(XPS_DIR) -f system.make clean
	-$(RM) $(XPS_DIR)/system.make $(HW_SPEC)
	-@echo ' '


####################################################
# H A R D W A R E
####################################################

$(XPS_DIR)/system.make :
	$(MAKE) -C $(XPS_DIR) all

$(XPS_DIR)/implementation/system.bit :
	$(MAKE) -C $(XPS_DIR) -f system.make bits

$(HW_SPEC)/system.xml :
	$(MAKE) -C $(XPS_DIR) -f system.make exporttosdk

.PHONY: bitstream
bitstream: $(XPS_DIR)/system.make $(XPS_DIR)/implementation/system.bit $(HW_SPEC)/system.xml


####################################################
# D O W N L O A D
####################################################
.PHONY: download-all
download-all: download-bits download-elf

.PHONY: download-bits
download-bits: $(XPS_DIR)/system.make
	$(MAKE) -C $(XPS_DIR) -f system.make download

.PHONY: download-elf
download-elf: app
	xmd -hw $(HW_SPEC)/system.xml -tcl download-elf.tcl $(EXECUTABLE) $(VERIFY_ELF)

-include $(OBJD)
