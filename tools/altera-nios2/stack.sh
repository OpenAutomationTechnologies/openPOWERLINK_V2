#!/bin/bash
################################################################################
#
# Stack library generator script for Altera Nios II
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

BSP_PATH=$1
CFLAGS="-Wall -Wextra -pedantic -std=c99"

if [ -z "${OPLK_BASE_DIR}" ];
then
    # Find oplk root path
    SCRIPTDIR=$(dirname $(readlink -f "${0}"))
    OPLK_BASE_DIR=$(readlink -f "${SCRIPTDIR}/../..")
fi

# process arguments
DEBUG=
OUT_PATH=.
while [ $# -gt 0 ]
do
    case "$1" in
        --debug)
            DEBUG=1
            ;;
        --out)
            shift
            OUT_PATH=$1
            ;;
        --help)
            echo "$ stack.sh [BSP] [OPTIONS]"
            echo "BSP           ... Path to generated BSP (settings.bsp, public.mk, Makefile)"
            echo "OPTIONS       ... :"
            echo "                      --debug ... Lib is generated with O0"
            echo "                      --out   ... Path to directory where Lib is generated to"
            exit 1
            ;;
        *)
            ;;
    esac
    shift
done

if [ -z "${BSP_PATH}" ];
then
    echo "ERROR: No BSP path is given!"
fi

# Get path to sopcinfo
echo "INFO: Get path to SOPCINFO file ... "

SOPCINFO_FILE=$(nios2-bsp-query-settings --settings ${BSP_PATH}/settings.bsp \
                            --cmd puts [get_sopcinfo_file] 2> /dev/null | grep sopcinfo)

if [ "$(expr substr $(uname -s) 1 6)" == "CYGWIN" ];
then
    # In cygwin convert returned path to posix style
    SOPCINFO_FILE=$(cygpath -u "${SOPCINFO_FILE}")
fi

echo "      ${SOPCINFO_FILE}"

if [ ! -f ${SOPCINFO_FILE} ];
then
    echo "ERROR: SOPCINFO file not found!"
    exit 1
fi

# Get path to board
HW_PATH=$(readlink -f "$(dirname ${SOPCINFO_FILE})/..")

if [ ! -d "${HW_PATH}" ];
then
    echo "ERROR: Path to SOPCINFO file does not exist (${HW_PATH})!"
    exit 1
fi

# Get path to common board directory
HW_COMMON_PATH=$(readlink -f "${HW_PATH}/../common")

if [ ! -d "${HW_COMMON_PATH}" ];
then
    echo "ERROR: Path to common board directory does not exist (${HW_COMMON_PATH})!"
    exit 1
fi

# Get path to board includes
BOARD_INCLUDE_PATH=$(readlink -f "${HW_PATH}/include")

if [ ! -d "${BOARD_INCLUDE_PATH}" ];
then
    echo "ERROR: Path to board include does not exist (${BOARD_INCLUDE_PATH})!"
    exit 1
fi

# Get bsp's cpu name
echo "INFO: Get CPU name ... "

CPU_NAME=$(nios2-bsp-query-settings --settings ${BSP_PATH}/settings.bsp \
                            --cmd puts [get_cpu_name] 2> /dev/null)

echo "      ${CPU_NAME}"

# Let's source the board.settings (null.settings before)
BOARD_SETTINGS_FILE=${HW_PATH}/board.settings
CFG_APP_CPU_NAME=
CFG_DRV_CPU_NAME=
CFG_OPENMAC=
CFG_HOSTINTERFACE=
CFG_NODE=
CFG_BOARD_CFLAGS=
if [ -f ${BOARD_SETTINGS_FILE} ]; then
    source ${BOARD_SETTINGS_FILE}
else
    echo "ERROR: ${BOARD_SETTINGS_FILE} not found!"
    exit 1
fi

# Let's find the stack library that fits best...
LIB_NAME=
LIB_SOURCES=
LIB_INCLUDES=
CFG_LIB_CFLAGS=
CFG_LIB_ARGS=
CFG_TCI_MEM_NAME=
if [ "${CPU_NAME}" == "${CFG_APP_CPU_NAME}" ]; then
    # The bsp's cpu matches to the app part
    CFG_TCI_MEM_NAME=${CFG_APP_TCI_MEM_NAME}
    if [ "${CFG_NODE}" == "CN" ] && [ -n "${CFG_OPENMAC}" ]; then
        LIB_NAME=oplkcn
        LIB_SOURCES=${HW_COMMON_PATH}/drivers/openmac/omethlib_phycfg.c
    elif [ "${CFG_NODE}" == "MN" ] && [ -n "${CFG_HOSTINTERFACE}" ]; then
        LIB_NAME=oplkmnapp-hostif
        LIB_SOURCES=
    fi
elif [ "${CPU_NAME}" == "${CFG_DRV_CPU_NAME}" ]; then
    # The bsp's cpu matches to the drv part
    CFG_TCI_MEM_NAME=${CFG_DRV_TCI_MEM_NAME}
    if [ "${CFG_NODE}" == "MN" ] && [ -n "${CFG_OPENMAC}" ] && [ -n "${CFG_HOSTINTERFACE}" ]; then
        echo "INFO: Compiling stack for Hostinterface design"
        LIB_NAME=oplkmndrv-hostif
        LIB_SOURCES=${HW_COMMON_PATH}/drivers/openmac/omethlib_phycfg.c
    elif [ "${CFG_NODE}" == "MN" ] && [ -n "${CFG_OPENMAC}" ] && [ -n "${CFG_DUALPROCSHM}" ]; then
        echo "INFO: Compiling stack for Dual Processor Shared memory design"
        LIB_NAME=oplkmndrv-dualprocshm
        LIB_SOURCES=${HW_COMMON_PATH}/drivers/openmac/omethlib_phycfg.c
    fi
else
    echo "ERROR: Please specify CFG_XXX_CPU_NAME in board.settings!"
    exit 1
fi

# Set TCI memory size
echo "INFO: Get TCI memory size ... "

TCI_MEM_SIZE=$(nios2-bsp-query-settings --settings ${BSP_PATH}/settings.bsp \
                            --cmd puts [get_addr_span ${CFG_TCI_MEM_NAME}] 2> /dev/null)

if [ -z "$TCI_MEM_SIZE" ]; then
    TCI_MEM_SIZE=0
fi

echo "      ${TCI_MEM_SIZE}"

# Let's source the stack library settings file
LIB_SETTINGS_FILE=${OPLK_BASE_DIR}/stack/build/altera-nios2/lib${LIB_NAME}/lib.settings
if [ -f ${LIB_SETTINGS_FILE}  ]; then
    source ${LIB_SETTINGS_FILE}
else
    echo "ERROR: ${LIB_SETTINGS_FILE} not found!"
fi

LIB_SOURCES+=" ${CFG_LIB_SOURCES}"
LIB_INCLUDES+=" ${CFG_LIB_INCLUDES} ${BOARD_INCLUDE_PATH}"

# Add design specific source files to the stack library
if [ "${CFG_NODE}" == "MN" ] && [ -n "${CFG_DUALPROCSHM}" ]; then
    if [ -z "${CFG_DRV_BUS}" ]; then
        # For internal bus usage between two processors using dualprocshm interface
        LIB_SOURCES+=" ${DUALPROCSHM_DRIVER_SOURCES}"
        CFG_LIB_CFLAGS+=" -D__INT_BUS__"
    elif [ "${CFG_DRV_BUS}" == "INT BUS" ]; then
        # For internal bus usage between two processors using dualprocshm interface
        LIB_SOURCES+=" ${DUALPROCSHM_DRIVER_SOURCES}"
        CFG_LIB_CFLAGS+=" -D__INT_BUS__"
    else
        echo "ERROR: Dualprocshm with ${CFG_DRV_BUS} interface for NIOS driver is not supported"
        exit 1
    fi
fi

if [ -n "${DEBUG}" ]; then
    LIB_OPT_LEVEL=-O0
    DEBUG_MODE=_DEBUG
else
    LIB_OPT_LEVEL=${CFG_LIB_OPT_LEVEL}
    DEBUG_MODE=NDEBUG
fi

OUT_PATH+=/lib${LIB_NAME}

LIB_GEN_ARGS="--lib-name ${LIB_NAME} --lib-dir ${OUT_PATH} \
--bsp-dir ${BSP_PATH} \
--src-files ${LIB_SOURCES} \
--set CFLAGS=${CFLAGS} ${CFG_LIB_CFLAGS} ${CFG_BOARD_CFLAGS} -D${DEBUG_MODE} -DCONFIG_${CFG_NODE} \
-DALT_TCIMEM_SIZE=${TCI_MEM_SIZE} \
--set LIB_CFLAGS_OPTIMIZATION=${LIB_OPT_LEVEL} \
${CFG_LIB_ARGS} \
"

for i in ${LIB_INCLUDES}
do
    LIB_GEN_ARGS+="--inc-dir ${i} "
done

nios2-lib-generate-makefile ${LIB_GEN_ARGS}

exit $?
