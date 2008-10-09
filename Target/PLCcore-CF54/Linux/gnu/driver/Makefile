
MODULENAME  = cf54drv
INSTALLDIR  = /home/tftpboot/daniel


# ------------------------- Secundary Call ----------------------------------
ifneq ($(KERNELRELEASE),) 

INCLUDES =		-I.

# Select here between debug and release settings
DBG_MODE =		NDEBUG
#DBG_MODE =		_DEBUG

MODFLAGS =		-D$(DBG_MODE)

EXTRA_CFLAGS =	$(INCLUDES) $(MODFLAGS)
#EXTRA_CFLAGS =	-Wa,-alhs $(INCLUDES) $(MODFLAGS)

cf54drv-objs :=	cf54basio.o


obj-m :=		cf54drv.o

cf54drv.o:		$(cf54drv-objs)
				@echo
				@echo
				pwd
				@echo
				@echo
				$(LD) -r -o $@ $(cf54drv-objs)





# ------------------------- Primary Call ------------------------------------
else

.EXPORT_ALL_VARIABLES:
# KDIR			:= /lib/modules/$(shell uname -r)/build
KDIR			:= $(M68K_LINUX_KDIR_PATH)
CROSS_COMPILE	:= $(M68K_CC_PREFIX)
LD_LIBRARY_PATH	:= $(M68K_LD_LIBRARY_PATH)
PWD				:= $(shell pwd)

BIN_TARGET		:= cf54drv.ko
BIN_DEST		:= /tftpboot

LIB_TARGET		:= cf54drv.a
LIB_DEST		:= ./


#           	Select here between debug and release settings
DBG_MODE		= NDEBUG
#DBG_MODE		= _DEBUG

#           	C-Compiler settings
CDEFS			= -D$(DBG_MODE)

#           	Toolchain command line settings
#           	Alternative option: -O2
#CFLAGS			+= -Os -Wall -mcfv4e $(INCLUDES) $(CDEFS)
CFLAGS			+= -O0 -g -Wall -mcfv4e $(INCLUDES) $(CDEFS)

CROSS		= 	$(M68K_CC_PREFIX)
LD_LIB_PATH	= 	$(M68K_LD_LIBRARY_PATH)
CC			= 	LD_LIBRARY_PATH=$(shell echo $(LD_LIB_PATH)) $(CROSS)gcc
AR			= 	LD_LIBRARY_PATH=$(shell echo $(LD_LIB_PATH)) $(CROSS)ar
STRIP		= 	LD_LIBRARY_PATH=$(shell echo $(LD_LIB_PATH)) $(CROSS)strip


call_driver_build:
				$(MAKE)	-C $(KDIR)	SUBDIRS=$(PWD) modules


				$(CC) $(CFLAGS) -c cf54drv.c -o cf54drv_lib.o
				$(AR) r cf54drv.a cf54drv_lib.o


.PHONY:     install
install:    call_driver_build
#			cp $(MODULENAME).ko $(INSTALLDIR)
			${M68K_CC_PREFIX}strip --strip-unneeded -o $(INSTALLDIR)/$(MODULENAME).ko $(MODULENAME).ko


endif



clean:
				rm -f *.bak
				rm -f *.tmp
				rm -f *~ *.o *.a
#				rm -f *.ko
				rm -f .built_in.o.cmd built_in.o
				rm -f .*.cmd *.mod.c
				rm -fr .tmp_versions/


