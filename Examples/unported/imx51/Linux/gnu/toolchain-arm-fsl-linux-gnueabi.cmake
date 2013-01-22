
set( CMAKE_SYSTEM_NAME Linux )
set( CMAKE_SYSTEM_PROCESSOR arm )
set( CMAKE_C_COMPILER arm-none-linux-gnueabi-gcc )
set( CMAKE_CXX_COMPILER arm-none-linux-gnueabi-g++ )
set( CMAKE_FIND_ROOT_PATH /opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi )

set( MAKE_KERNEL_ARCH arm )
set( MAKE_KERNEL_CROSS_COMPILE ${CMAKE_FIND_ROOT_PATH}/bin/arm-none-linux-gnueabi- )

