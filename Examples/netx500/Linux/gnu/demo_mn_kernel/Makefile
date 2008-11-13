
INSTALLDIR  = /home/tftpboot/daniel

MODULENAME = epl

# ------------------------- Secondary Call ----------------------------------
ifneq ($(KERNELRELEASE),)

MAINPATH	= $(obj)/../../../../..
INCLUDES	= -I$(MAINPATH)/Include \
			  -I$(MAINPATH)/Edrv/netx_ethhub/include \
			  -I$(obj) \
			  -I$(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/generic \
			  -I$(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/driver \
			  -I$(MAINPATH)/SharedBuff \
			  -I$(MAINPATH)/ObjDicts/Api_CN
#			  -I$(MAINPATH)/Target/PLCcore-CF54/Linux/gnu/driver

MODULEOBJS	= demo_main.o \
			  EdrvNetx.o \
			  EplDllk.o EplDllkCal.o \
			  EplDlluCal.o \
			  EplEventk.o EplEventu.o \
			  EplNmtk.o EplNmtkCal.o \
			  EplNmtu.o EplNmtuCal.o EplNmtCnu.o \
			  EplPdok.o EplPdokCal.o \
			  EplObdk.o \
			  EplSdoUdpu.o EplSdoComu.o EplSdoAsySequ.o EplSdoAsndu.o \
			  EplTimeruLinuxKernel.o \
			  EplErrorHandlerk.o \
			  EplApiGeneric.o \
			  Objdict.o \
			  amiarm.o \
			  VirtualEthernetLinux.o \
			  SharedBuff.o ShbIpc-LinuxKernel.o \
			  SocketLinuxKernel.o \
			  TimerHighReskX86.o \
			  proc_fs.o \
			  EplNmtMnu.o EplIdentu.o EplStatusu.o
#			  EplObdu.o EplObdkCal.o EplObduCal.o

#           Select here between debug and release settings
#DBG_MODE = NDEBUG
DBG_MODE = _DEBUG

#MODFLAGS =	-D$(DBG_MODE)
#MODFLAGS =	-D$(DBG_MODE) -D_DBG_TRACE_POINTS_ -DDEF_DEBUG_LVL=0xEC000040L
MODFLAGS =	-D$(DBG_MODE) -D_DBG_TRACE_POINTS_ -DDEF_DEBUG_LVL=0xEC000000L
#MODFLAGS =	-D$(DBG_MODE) -DDEF_DEBUG_LVL=0xEC000000L

EXTRA_CFLAGS = $(INCLUDES) $(MODFLAGS)
#EXTRA_CFLAGS = -Wa,-alhs $(INCLUDES) $(MODFLAGS)

$(MODULENAME)-objs := $(MODULEOBJS)

obj-m := $(MODULENAME).o



# ------------------------- Primary Call ------------------------------------
else

.EXPORT_ALL_VARIABLES:
KDIR			:= /lib/modules/$(shell uname -r)/build

# we crosscompile for ARM
ARCH := arm
CROSS_COMPILE := arm-v4t-linux-gnueabi-

PWD				:= $(shell pwd)


.PHONY:     call_driver_build
call_driver_build:
			$(MAKE)	-C $(KDIR)	SUBDIRS=$(PWD) modules

#.PHONY:     install
#install:    call_driver_build
#			cp $(MODULENAME).ko $(INSTALLDIR)

endif


.PHONY:     clean
clean:
			rm -f *.bak
			rm -f *.tmp
			rm -f *~ *.o *.ko
			rm -f .built_in.o.cmd built_in.o Module.symvers
			rm -f .*.cmd *.ko *.mod.c
			rm -fr .tmp_versions/


