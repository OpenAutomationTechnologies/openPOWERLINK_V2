################################################################################
#
# File lists for openPOWERLINK stack sources
#
# Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

################################################################################
# Common sources
################################################################################

SET(COMMON_SOURCES
    ${COMMON_SOURCE_DIR}/debugstr.c
    )

SET(COMMON_WINDOWS_SOURCES
    ${CONTRIB_SOURCE_DIR}/trace/trace-windows.c
    )

SET(COMMON_LINUXUSER_SOURCES
    ${ARCH_SOURCE_DIR}/linux/ftracedebug.c
    ${CONTRIB_SOURCE_DIR}/trace/trace-printf.c
    )

SET(COMMON_LINUXKERNEL_SOURCES
    ${CONTRIB_SOURCE_DIR}/trace/trace-printk.c
    )

SET(COMMON_NOOS_SOURCES
    ${CONTRIB_SOURCE_DIR}/trace/trace-printf.c
    )

SET(COMMON_CAL_DIRECT_SOURCES
    ${COMMON_SOURCE_DIR}/dll/dllcal-direct.c
    )

################################################################################
# Application library (User) sources
################################################################################

################################################################################
# MN specific sources

SET(USER_MN_SOURCES
    ${USER_SOURCE_DIR}/obd/obdcdc.c
    ${USER_SOURCE_DIR}/api/processimage-cia302.c
    ${USER_SOURCE_DIR}/cfmu.c
    )

################################################################################
# Generic sources

SET(USER_SOURCES
    ${USER_SOURCE_DIR}/api/generic.c
    ${USER_SOURCE_DIR}/api/processimage.c
    ${USER_SOURCE_DIR}/api/sdotest.c
    ${USER_SOURCE_DIR}/obd/obd.c
    ${USER_SOURCE_DIR}/obd/obdcreate.c
    ${USER_SOURCE_DIR}/dll/dllucal.c
    ${USER_SOURCE_DIR}/event/eventu.c
    ${USER_SOURCE_DIR}/nmt/nmtu.c
    ${USER_SOURCE_DIR}/nmt/nmtcnu.c
    ${USER_SOURCE_DIR}/nmt/nmtmnu.c
    ${USER_SOURCE_DIR}/nmt/identu.c
    ${USER_SOURCE_DIR}/nmt/statusu.c
    ${USER_SOURCE_DIR}/nmt/syncu.c
    ${USER_SOURCE_DIR}/pdo/pdou.c
    ${USER_SOURCE_DIR}/pdo/pdoucal.c
    ${USER_SOURCE_DIR}/pdo/pdoucal-triplebufshm.c
    ${USER_SOURCE_DIR}/sdo/sdotest-com.c
    ${USER_SOURCE_DIR}/sdo/sdotest-seq.c
    ${USER_SOURCE_DIR}/sdo/sdocom-dummy.c
    ${USER_SOURCE_DIR}/sdo/sdocom.c
    ${USER_SOURCE_DIR}/sdo/sdocom-std.c
    ${USER_SOURCE_DIR}/sdo/sdoseq.c
    ${USER_SOURCE_DIR}/sdo/sdoasnd.c
    ${USER_SOURCE_DIR}/sdo/sdoudp.c
    ${USER_SOURCE_DIR}/errhnd/errhndu.c
    ${USER_SOURCE_DIR}/ctrl/ctrlu.c
    ${USER_SOURCE_DIR}/ledu.c
    )

################################################################################
# User control CAL sources

SET(CTRL_UCAL_LINUXIOCTL_SOURCES
    ${USER_SOURCE_DIR}/ctrl/ctrlucal-ioctl.c
    )

SET(CTRL_UCAL_POSIXMEM_SOURCES
    ${USER_SOURCE_DIR}/ctrl/ctrlucal-mem.c
    ${COMMON_SOURCE_DIR}/ctrl/ctrlcal-posixshm.c
    )

SET(CTRL_UCAL_DIRECT_SOURCES
    ${USER_SOURCE_DIR}/ctrl/ctrlucal-direct.c
    )

SET(CTRL_UCAL_HOSTIF_SOURCES
    ${USER_SOURCE_DIR}/ctrl/ctrlucal-hostif.c
    )

SET(CTRL_UCAL_DUALPROCSHM_SOURCES
    ${USER_SOURCE_DIR}/ctrl/ctrlucal-noosdual.c
    )

################################################################################
# User DLL CAL sources

SET(DLL_UCAL_CIRCBUF_SOURCES
    ${USER_SOURCE_DIR}/dll/dllucal-circbuf.c
    )

SET(DLL_UCAL_LINUXIOCTL_SOURCES
    ${USER_SOURCE_DIR}/dll/dllucal-ioctl.c
    )

################################################################################
# User error handler CAL sources

SET(ERRHND_UCAL_LINUXIOCTL_SOURCES
    ${USER_SOURCE_DIR}/errhnd/errhnducal-ioctl.c
    )

SET(ERRHND_UCAL_POSIXMEM_SOURCES
    ${USER_SOURCE_DIR}/errhnd/errhnducal-posixshm.c
    )

SET(ERRHND_UCAL_LOCAL_SOURCES
    ${USER_SOURCE_DIR}/errhnd/errhnducal-local.c
    )

SET(ERRHND_UCAL_HOSTIF_SOURCES
    ${USER_SOURCE_DIR}/errhnd/errhnducal-hostif.c
    )

SET(ERRHND_UCAL_DUALPROCSHM_SOURCES
    ${USER_SOURCE_DIR}/errhnd/errhnducal-noosdual.c
    )

################################################################################
# User event CAL sources

SET(EVENT_UCAL_LINUXUSER_SOURCES
    ${USER_SOURCE_DIR}/event/eventucal-linux.c
    ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
    )

SET(EVENT_UCAL_LINUXIOCTL_SOURCES
    ${USER_SOURCE_DIR}/event/eventucal-linuxioctl.c
    )

SET(EVENT_UCAL_WINDOWS_SOURCES
    ${USER_SOURCE_DIR}/event/eventucal-win32.c
    ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
    )

SET(EVENT_UCAL_NOOSKERNEL_SOURCES
    ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
    ${USER_SOURCE_DIR}/event/eventucal-nooscircbuf.c
    )

SET(EVENT_UCAL_NOOSHOSTIF_SOURCES
    ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
    ${USER_SOURCE_DIR}/event/eventucal-nooshostif.c
    )

SET(EVENT_UCAL_DUALPROCSHM_SOURCES
    ${USER_SOURCE_DIR}/event/eventucal-noosdual.c
    ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
    )

################################################################################
# User PDO CAL sources
SET(PDO_UCAL_LOCAL_SOURCES
    ${USER_SOURCE_DIR}/pdo/pdoucalmem-local.c
    ${USER_SOURCE_DIR}/pdo/pdoucalsync-null.c
    )

SET(PDO_UCAL_POSIX_SOURCES
    ${USER_SOURCE_DIR}/pdo/pdoucalmem-posixshm.c
    ${USER_SOURCE_DIR}/pdo/pdoucalsync-bsdsem.c
    )

SET(PDO_UCAL_LINUXMMAPIOCTL_SOURCES
    ${USER_SOURCE_DIR}/pdo/pdoucalsync-ioctl.c
    ${USER_SOURCE_DIR}/pdo/pdoucalmem-linuxmmap.c
    )

SET(PDO_UCAL_HOSTIF_SOURCES
    ${USER_SOURCE_DIR}/pdo/pdoucalsync-hostif.c
    ${USER_SOURCE_DIR}/pdo/pdoucalmem-hostif.c
    )

SET(PDO_UCAL_DUALPROCSHM_SOURCES
    ${USER_SOURCE_DIR}/pdo/pdoucalmem-noosdual.c
    ${USER_SOURCE_DIR}/pdo/pdoucalsync-noosdual.c
    )

################################################################################
# Driver library (Kernel) sources
################################################################################

################################################################################
# Generic driver library sources

SET(KERNEL_SOURCES
    ${KERNEL_SOURCE_DIR}/dll/dllk.c
    ${KERNEL_SOURCE_DIR}/dll/dllkfilter.c
    ${KERNEL_SOURCE_DIR}/dll/dllkstatemachine.c
    ${KERNEL_SOURCE_DIR}/dll/dllkevent.c
    ${KERNEL_SOURCE_DIR}/dll/dllkframe.c
    ${KERNEL_SOURCE_DIR}/dll/dllknode.c
    ${KERNEL_SOURCE_DIR}/dll/dllkcal.c
    ${KERNEL_SOURCE_DIR}/event/eventk.c
    ${KERNEL_SOURCE_DIR}/nmt/nmtk.c
    ${KERNEL_SOURCE_DIR}/pdo/pdok.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcal.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcal-triplebufshm.c
    ${KERNEL_SOURCE_DIR}/pdo/pdoklut.c
    ${KERNEL_SOURCE_DIR}/errhnd/errhndk.c
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlk.c
    )

################################################################################
# Control CAL sources

SET(CTRL_KCAL_POSIXMEM_SOURCES
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-mem.c
    ${COMMON_SOURCE_DIR}/ctrl/ctrlcal-posixshm.c
    )

SET(CTRL_KCAL_DIRECT_SOURCES
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-direct.c
    )

SET(CTRL_KCAL_LINUXIOCTL_SOURCES
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-direct.c
    )

SET(CTRL_KCAL_HOSTIF_SOURCES
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-hostif.c
    )

SET(CTRL_KCAL_DUALPROCSHM_SOURCES
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-noosdual.c
    )

################################################################################
# Kernel DLL CAL sources

SET(DLL_KCAL_CIRCBUF_SOURCES
    ${KERNEL_SOURCE_DIR}/dll/dllkcal-circbuf.c
    )

SET(DLL_KCAL_LINUXIOCTL_SOURCES
    ${KERNEL_SOURCE_DIR}/dll/dllkcal-circbuf.c
    )

################################################################################
# Kernel error handler CAL sources

SET(ERRHND_KCAL_POSIXMEM_SOURCES
    ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-posixshm.c
    )

SET(ERRHND_KCAL_LOCAL_SOURCES
    ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-local.c
    )

SET(ERRHND_KCAL_HOSTIF_SOURCES
    ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-hostif.c
    )

SET(ERRHND_KCAL_DUALPROCSHM_SOURCES
    ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-noosdual.c
    )

################################################################################
# Kernel event CAL sources

SET(EVENT_KCAL_LINUXUSER_SOURCES
    ${KERNEL_SOURCE_DIR}/event/eventkcal-linux.c
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    )

SET(EVENT_KCAL_WINDOWS_SOURCES
    ${KERNEL_SOURCE_DIR}/event/eventkcal-win32.c
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    )

SET(EVENT_KCAL_LINUXKERNEL_SOURCES
    ${KERNEL_SOURCE_DIR}/event/eventkcal-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    )

SET(EVENT_KCAL_NOOSKERNEL_SOURCES
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    ${KERNEL_SOURCE_DIR}/event/eventkcal-nooscircbuf.c
    )

SET(EVENT_KCAL_NOOSHOSTIF_SOURCES
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    ${KERNEL_SOURCE_DIR}/event/eventkcal-nooshostif.c
    )

SET(EVENT_KCAL_DUALPROCSHM_SOURCES
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    ${KERNEL_SOURCE_DIR}/event/eventkcal-noosdual.c
    )

################################################################################
# Kernel PDO CAL sources

SET(PDO_KCAL_LOCAL_SOURCES
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-local.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-null.c
    )

SET(PDO_KCAL_POSIXMEM_SOURCES
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-posixshm.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-bsdsem.c
    )

SET(PDO_KCAL_LINUXKERNEL_SOURCES
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-linuxkernel.c
    )

SET(PDO_KCAL_HOSTIF_SOURCES
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-hostif.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-hostif.c
    )

SET(PDO_KCAL_DUALPROCSHM_SOURCES
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-noosdual.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-noosdual.c
    )

################################################################################
# Kernel Ethernet

SET(HARDWARE_DRIVER_LINUXUSER_SOURCES
    ${KERNEL_SOURCE_DIR}/veth/veth-linuxuser.c
    ${KERNEL_SOURCE_DIR}/timer/hrestimer-posix.c
    ${EDRV_SOURCE_DIR}/edrvcyclic.c
    ${EDRV_SOURCE_DIR}/edrv-pcap_linux.c
    )

SET(HARDWARE_DRIVER_WINDOWS_SOURCES
    ${EDRV_SOURCE_DIR}/edrvcyclic.c
    ${EDRV_SOURCE_DIR}/edrv-pcap_win.c
    ${KERNEL_SOURCE_DIR}/timer/hrestimer-windows.c
    )

SET(HARDWARE_DRIVER_LINUXKERNEL_SOURCES
     ${KERNEL_SOURCE_DIR}/veth/veth-linuxkernel.c
     ${KERNEL_SOURCE_DIR}/timer/hrestimer-linuxkernel.c
     ${EDRV_SOURCE_DIR}/edrvcyclic.c
     )

SET(HARDWARE_DRIVER_OPENMAC_SOURCES
     ${KERNEL_SOURCE_DIR}/timer/timestamp-openmac.c
     ${KERNEL_SOURCE_DIR}/edrv/edrv-openmac.c
     ${KERNEL_SOURCE_DIR}/veth/veth-generic.c
     )

SET(HARDWARE_DRIVER_OPENMAC_CN_SOURCES
     ${KERNEL_SOURCE_DIR}/timer/synctimer-openmac.c
     )

SET(HARDWARE_DRIVER_OPENMAC_MN_SOURCES
    ${KERNEL_SOURCE_DIR}/timer/hrestimer-openmac.c
    ${KERNEL_SOURCE_DIR}/edrv/edrvcyclic-openmac.c
    )
################################################################################
# User timer sources

# Linux userspace sources
SET(USER_TIMER_LINUXUSER_SOURCES
    ${USER_SOURCE_DIR}/timer/timer-linuxuser.c
    )

SET(USER_TIMER_WINDOWS_SOURCES
    ${USER_SOURCE_DIR}/timer/timer-generic.c
    )

SET(USER_TIMER_GENERIC_SOURCES
    ${USER_SOURCE_DIR}/timer/timer-generic.c
    )

################################################################################
# Circular buffer sources
################################################################################

SET(CIRCBUF_POSIX_SOURCES
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-posixshm.c
    )

SET(CIRCBUF_WINDOWS_SOURCES
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-win32.c
    )

SET(CIRCBUF_LINUXKERNEL_SOURCES
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-linuxkernel.c
    )

SET(CIRCBUF_NOOS_SOURCES
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-noos.c
    )

SET(CIRCBUF_NOOSHOSTIF_SOURCES
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-nooshostif.c
    )

SET(CIRCBUF_DUALPROCSHM_SOURCES
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-noosdual.c
    )

################################################################################
# Memory Mapping sources
################################################################################

SET(MEMMAP_NOOSLOCAL_SOURCES
    ${COMMON_SOURCE_DIR}/memmap/memmap-nooslocal.c
    )

SET(MEMMAP_NOOSHOSTIF_SOURCES
    ${COMMON_SOURCE_DIR}/memmap/memmap-nooshostif.c
    )

SET(MEMMAP_NULL_SOURCES
    ${COMMON_SOURCE_DIR}/memmap/memmap-null.c
    )

SET(MEMMAP_DUALPROCSHM_SOURCES
    ${COMMON_SOURCE_DIR}/memmap/memmap-noosdual.c
    )

################################################################################
# Target system specific sources
################################################################################

SET(TARGET_WINDOWS_SOURCES
    ${ARCH_SOURCE_DIR}/windows/target-windows.c
    ${ARCH_SOURCE_DIR}/windows/target-mutex.c
    )

SET(TARGET_LINUX_SOURCES
    ${ARCH_SOURCE_DIR}/linux/target-linux.c
    ${ARCH_SOURCE_DIR}/linux/target-mutex.c
    )

SET(TARGET_MICROBLAZE_SOURCES
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/systemtimer.c
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/usleep.c
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/target-microblaze.c
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/target-mutex.c
    )

SET(TARGET_MICROBLAZE_LOCAL_SOURCES
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/lock-localnoos.c
    )

SET(TARGET_MICROBLAZE_DUAL_SOURCES
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/lock-dualprocnoos.c
    )

SET(TARGET_MICROBLAZE_OPENMAC_SOURCES
    ${ARCH_SOURCE_DIR}/xilinx_microblaze/openmac-microblaze.c
    )

SET(TARGET_XILINX_ARM_SOURCES
    ${ARCH_SOURCE_DIR}/xilinx-zynqarm/target-zynqarm.c
    ${ARCH_SOURCE_DIR}/xilinx-zynqarm/target-mutex.c
    )

SET(TARGET_XILINX_ARM_DUAL_SOURCES
    ${ARCH_SOURCE_DIR}/xilinx-zynqarm/lock-dualprocnoos.c
    )

SET(TARGET_ALTERA_ARM_SOURCES
    ${ARCH_SOURCE_DIR}/altera-c5socarm/target-c5socarm.c
    ${ARCH_SOURCE_DIR}/altera-c5socarm/target-mutex.c
    ${ARCH_SOURCE_DIR}/altera-c5socarm/sleep.c
    )

SET(TARGET_ALTERA_ARM_DUAL_SOURCES
    ${ARCH_SOURCE_DIR}/altera-c5socarm/lock-dualprocnoos.c
    )
################################################################################
# Architecture specific sources
################################################################################

SET(ARCH_X86_SOURCES
    ${COMMON_SOURCE_DIR}/ami/amix86.c
    )

SET(ARCH_LE_SOURCES
    ${COMMON_SOURCE_DIR}/ami/amile.c
    )

################################################################################
# Header Files
################################################################################

SET(OPLK_HEADERS
    ${STACK_INCLUDE_DIR}/oplk/benchmark.h
    ${STACK_INCLUDE_DIR}/oplk/cfm.h
    ${STACK_INCLUDE_DIR}/oplk/debug.h
    ${STACK_INCLUDE_DIR}/oplk/debugstr.h
    ${STACK_INCLUDE_DIR}/oplk/dll.h
    ${STACK_INCLUDE_DIR}/oplk/oplk.h
    ${STACK_INCLUDE_DIR}/oplk/oplkdefs.h
    ${STACK_INCLUDE_DIR}/oplk/errordefs.h
    ${STACK_INCLUDE_DIR}/oplk/frame.h
    ${STACK_INCLUDE_DIR}/oplk/oplkinc.h
    ${STACK_INCLUDE_DIR}/oplk/targetsystem.h
    ${STACK_INCLUDE_DIR}/oplk/version.h
    ${STACK_INCLUDE_DIR}/oplk/event.h
    ${STACK_INCLUDE_DIR}/oplk/basictypes.h
    ${STACK_INCLUDE_DIR}/oplk/led.h
    ${STACK_INCLUDE_DIR}/oplk/nmt.h
    ${STACK_INCLUDE_DIR}/oplk/obd.h
    ${STACK_INCLUDE_DIR}/oplk/obdcdc.h
    ${STACK_INCLUDE_DIR}/oplk/obdmacro.h
    ${STACK_INCLUDE_DIR}/oplk/powerlink-module.h
    ${STACK_INCLUDE_DIR}/oplk/sdo.h
    ${STACK_INCLUDE_DIR}/oplk/sdoabortcodes.h
    ${STACK_INCLUDE_DIR}/oplk/section-default.h
    ${STACK_INCLUDE_DIR}/oplk/section-microblaze.h
    ${STACK_INCLUDE_DIR}/oplk/section-nios2.h
    ${STACK_INCLUDE_DIR}/oplk/targetdefs/nios2.h
    ${STACK_INCLUDE_DIR}/oplk/targetdefs/microblaze.h
    ${STACK_INCLUDE_DIR}/oplk/targetdefs/linux.h
    ${STACK_INCLUDE_DIR}/oplk/targetdefs/windows.h
    ${STACK_INCLUDE_DIR}/oplk/targetdefs/wince.h
    ${STACK_INCLUDE_DIR}/oplk/targetdefs/vxworks.h
    )

SET(STACK_HEADERS
    ${STACK_INCLUDE_DIR}/common/ami.h
    ${STACK_INCLUDE_DIR}/common/circbuffer.h
    ${STACK_INCLUDE_DIR}/common/ctrl.h
    ${STACK_INCLUDE_DIR}/common/ctrlcal.h
    ${STACK_INCLUDE_DIR}/common/ctrlcal-mem.h
    ${STACK_INCLUDE_DIR}/common/defaultcfg.h
    ${STACK_INCLUDE_DIR}/common/dllcal.h
    ${STACK_INCLUDE_DIR}/common/errhnd.h
    ${STACK_INCLUDE_DIR}/common/oplkinc.h
    ${STACK_INCLUDE_DIR}/common/pdo.h
    ${STACK_INCLUDE_DIR}/common/target.h
    ${STACK_INCLUDE_DIR}/common/ftracedebug.h
    ${STACK_INCLUDE_DIR}/common/timer.h
    )

SET(USER_HEADERS
    ${STACK_INCLUDE_DIR}/user/cfmu.h
    ${STACK_INCLUDE_DIR}/user/ctrlu.h
    ${STACK_INCLUDE_DIR}/user/ctrlucal.h
    ${STACK_INCLUDE_DIR}/user/dllucal.h
    ${STACK_INCLUDE_DIR}/user/errhndu.h
    ${STACK_INCLUDE_DIR}/user/eventu.h
    ${STACK_INCLUDE_DIR}/user/eventucal.h
    ${STACK_INCLUDE_DIR}/user/eventucalintf.h
    ${STACK_INCLUDE_DIR}/user/identu.h
    ${STACK_INCLUDE_DIR}/user/ledu.h
    ${STACK_INCLUDE_DIR}/user/nmtcnu.h
    ${STACK_INCLUDE_DIR}/user/nmtmnu.h
    ${STACK_INCLUDE_DIR}/user/nmtu.h
    ${STACK_INCLUDE_DIR}/user/pdou.h
    ${STACK_INCLUDE_DIR}/user/pdoucal.h
    ${STACK_INCLUDE_DIR}/user/sdocom.h
    ${STACK_INCLUDE_DIR}/user/sdotest.h
    ${STACK_INCLUDE_DIR}/user/sdoseq.h
    ${STACK_INCLUDE_DIR}/user/sdoal.h
    ${STACK_INCLUDE_DIR}/user/sdoasnd.h
    ${STACK_INCLUDE_DIR}/user/sdoudp.h
    ${STACK_INCLUDE_DIR}/user/statusu.h
    ${STACK_INCLUDE_DIR}/user/syncu.h
    ${STACK_INCLUDE_DIR}/user/timeru.h
    )

SET(KERNEL_HEADERS
    ${STACK_INCLUDE_DIR}/kernel/ctrlk.h
    ${STACK_INCLUDE_DIR}/kernel/ctrlkcal.h
    ${STACK_INCLUDE_DIR}/kernel/dllk.h
    ${STACK_INCLUDE_DIR}/kernel/dllkcal.h
    ${STACK_INCLUDE_DIR}/kernel/dllkfilter.h
    ${STACK_INCLUDE_DIR}/kernel/dllktgt.h
    ${STACK_INCLUDE_DIR}/kernel/hrestimer.h
    ${STACK_INCLUDE_DIR}/kernel/timestamp.h
    ${STACK_INCLUDE_DIR}/kernel/synctimer.h
    ${STACK_INCLUDE_DIR}/kernel/errhndk.h
    ${STACK_INCLUDE_DIR}/kernel/eventk.h
    ${STACK_INCLUDE_DIR}/kernel/eventkcal.h
    ${STACK_INCLUDE_DIR}/kernel/eventkcalintf.h
    ${STACK_INCLUDE_DIR}/kernel/nmtk.h
    ${STACK_INCLUDE_DIR}/kernel/pdok.h
    ${STACK_INCLUDE_DIR}/kernel/pdokcal.h
    ${STACK_INCLUDE_DIR}/kernel/pdoklut.h
    ${STACK_INCLUDE_DIR}/kernel/veth.h
    ${STACK_INCLUDE_DIR}/kernel/edrv.h
    ${STACK_INCLUDE_DIR}/kernel/edrvcyclic.h
    )

SET(OBJDICT_HEADERS
    ${OBJDICT_DIR}/generic/objdict_1000-13ff.h
    ${OBJDICT_DIR}/generic/objdict_1b00-1fff.h
    )
