
INSTALLDIR  = /tftpboot

MODULENAME = epl

# ------------------------- Secondary Call ----------------------------------
ifneq ($(KERNELRELEASE),)

MAINPATH	:= ../../../../..
MAINDIR	:= $(src)/$(MAINPATH)
INCLUDES	:= -I$(MAINDIR)/Include \
			  -I$(MAINDIR)/Edrv/fec_548x/include/net \
			  -I$(src) \
			  -I$(MAINDIR)/Target/PLCcore-CF54/Linux/gnu/generic \
			  -I$(MAINDIR)/Target/PLCcore-CF54/Linux/gnu/driver \
			  -I$(MAINDIR)/SharedBuff \
			  -I$(MAINDIR)/ObjDicts/Api_CN

MODULEOBJS	= demo_main.o \
			  $(MAINPATH)/Edrv/fec_548x/net/fec.o $(MAINPATH)/Edrv/fec_548x/net/ks8721.o $(MAINPATH)/Edrv/fec_548x/net/ksz8893.o \
			  $(MAINPATH)/EplStack/EplDllk.o $(MAINPATH)/EplStack/EplDllkCal.o \
			  $(MAINPATH)/EplStack/EplDlluCal.o \
			  $(MAINPATH)/EplStack/EplLedu.o \
			  $(MAINPATH)/EplStack/EplEventk.o $(MAINPATH)/EplStack/EplEventu.o \
			  $(MAINPATH)/EplStack/EplNmtk.o $(MAINPATH)/EplStack/EplNmtkCal.o \
			  $(MAINPATH)/EplStack/EplNmtu.o $(MAINPATH)/EplStack/EplNmtuCal.o $(MAINPATH)/EplStack/EplNmtCnu.o \
			  $(MAINPATH)/EplStack/EplPdok.o $(MAINPATH)/EplStack/EplPdokCal.o \
			  $(MAINPATH)/EplStack/EplPdou.o $(MAINPATH)/EplStack/EplPdouCal.o \
			  $(MAINPATH)/EplStack/EplObd.o \
			  $(MAINPATH)/EplStack/EplSdoUdpu.o $(MAINPATH)/EplStack/EplSdoComu.o $(MAINPATH)/EplStack/EplSdoAsySequ.o $(MAINPATH)/EplStack/EplSdoAsndu.o \
			  $(MAINPATH)/EplStack/EplTimeruLinuxKernel.o \
			  $(MAINPATH)/EplStack/EplErrorHandlerk.o \
			  $(MAINPATH)/EplStack/EplApiGeneric.o \
			  $(MAINPATH)/ObjDicts/Api_CN/Objdict.o \
			  $(MAINPATH)/EplStack/amibe.o \
			  $(MAINPATH)/EplStack/VirtualEthernetLinux.o \
			  $(MAINPATH)/SharedBuff/SharedBuff.o $(MAINPATH)/SharedBuff/ShbIpc-LinuxKernel.o \
			  $(MAINPATH)/EplStack/SocketLinuxKernel.o \
			  $(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/generic/TimerHighReskMcf54xx.o \
			  $(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/generic/target_mcf548x.o $(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/generic/proc_fs.o

#           Select here between debug and release settings
#DBG_MODE = NDEBUG
DBG_MODE = _DEBUG

#MODFLAGS =	-D$(DBG_MODE)
#MODFLAGS =	-D$(DBG_MODE) -D_DBG_TRACE_POINTS_ -DDEF_DEBUG_LVL=0xEC000040L
MODFLAGS =	-D$(DBG_MODE) -D_DBG_TRACE_POINTS_ -DDEF_DEBUG_LVL=0xEC000000L

EXTRA_CFLAGS = $(INCLUDES) $(MODFLAGS)
#EXTRA_CFLAGS = -Wa,-alhs $(INCLUDES) $(MODFLAGS)

obj-m := $(MODULENAME).o

$(MODULENAME)-y := $(MODULEOBJS)


# add the object files of the stack to the variable clean-files,
# because the object files in upper directories were not deleted otherwise.
clean-files := $(MODULEOBJS) \
		$(foreach f,$(MODULEOBJS),$(dir $(f)).$(notdir $(f)).cmd)


# ------------------------- Primary Call ------------------------------------
else

.EXPORT_ALL_VARIABLES:
#KDIR			:= /lib/modules/$(shell uname -r)/build
KDIR			:= $(M68K_LINUX_KDIR_PATH)
CROSS_COMPILE	:= $(M68K_CC_PREFIX)
LD_LIBRARY_PATH	:= $(M68K_LD_LIBRARY_PATH)


PWD				:= $(shell pwd)
MAINPATH	= ../../../../..
# directory with scripts
SCRIPTDIR	= $(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/scripts/
# directory with board driver for ECUcore-5484
DRIVERDIR	= $(PWD)/$(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/driver/

.PHONY:     call_driver_build
call_driver_build:
			$(MAKE)	-C $(KDIR)	SUBDIRS=$(PWD) modules

.PHONY:     install
install:    call_driver_build
#			cp $(MODULENAME).ko $(INSTALLDIR)
			${M68K_CC_PREFIX}strip --strip-unneeded -o $(INSTALLDIR)/$(MODULENAME).ko $(MODULENAME).ko

# build the installer
.PHONY:     installer
installer:	call_driver_build
# build board driver
			$(MAKE) -C $(DRIVERDIR) INSTALLDIR=$(PWD) install
# build installer
	                dos2unix $(SCRIPTDIR)build-install.sh && chmod +x $(SCRIPTDIR)build-install.sh && $(SCRIPTDIR)build-install.sh


.PHONY:     clean
clean:
			$(MAKE)	-C $(KDIR)	SUBDIRS=$(PWD) clean
#			rm -f *.bak
#			rm -f *.tmp
#			rm -f *~ *.o *.ko
#			rm -f .built_in.o.cmd built_in.o
#			rm -f .*.cmd *.ko *.mod.c
#			rm -fr .tmp_versions/

endif

