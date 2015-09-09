
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

#### LIB ####

IF (CFG_OPLK_LIB)
    ADD_SUBDIRECTORY("stack")
ENDIF (CFG_OPLK_LIB)

#### Linux kernel module  ####

IF (CFG_KERNEL_DRIVERS)
    ADD_SUBDIRECTORY("drivers/linux/drv_kernelmod_edrv")
ENDIF (CFG_KERNEL_DRIVERS)

#### Pcap userspace driver  ####

IF (CFG_PCAP_DAEMON)
    ADD_SUBDIRECTORY("drivers/linux/drv_daemon_pcap")
ENDIF (CFG_PCAP_DAEMON)

#### OpenPowerLink Demos  ####

# Add subdirectory of CN console demo application
IF (CFG_DEMO_CN_CONSOLE)
    ADD_SUBDIRECTORY("apps/demo_cn_console")
ENDIF (CFG_DEMO_CN_CONSOLE)

# Add subdirectory of MN console demo application
IF (CFG_DEMO_MN_CONSOLE)
    ADD_SUBDIRECTORY("apps/demo_mn_console")
ENDIF (CFG_DEMO_MN_CONSOLE)

IF (CFG_DEMO_MN_QT)
    ADD_SUBDIRECTORY("apps/demo_mn_qt")
ENDIF (CFG_DEMO_MN_QT)
