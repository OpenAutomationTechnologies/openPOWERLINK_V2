#!/bin/bash
################################################################################
#
# Drivers generator script for Altera Nios II
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

DRV_PATH=$1
BOARD_PATH=$2
CFLAGS="-Wall -Wextra -pedantic -std=c99"

if [ -z "${OPLK_BASE_DIR}" ];
then
    # Find oplk root path
    SCRIPTDIR=$(dirname $(readlink -f "${0}"))
    OPLK_BASE_DIR=$(readlink -f "${SCRIPTDIR}/../..")
fi

# process arguments
DEBUG=
DEBUG_ARG=
OUT_PATH=.
while [ $# -gt 0 ]
do
    case "$1" in
        --debug)
            DEBUG=1
            DEBUG_ARG=$1
            ;;
        --out)
            shift
            OUT_PATH=$1
            ;;
        --help)
            echo "$ drivers.sh [DRV] [BOARD] [OPTIONS]"
            echo "DRV       ... Path to drivers project (drivers.settings)"
            echo "BOARD     ... Path to hardware project (board.settings)"
            echo "OPTIONS   ... :"
            echo "              --debug ... Lib is generated with O0"
            echo "              --out   ... Path to output directory"
            exit 1
            ;;
        *)
            ;;
    esac
    shift
done

if [ -z "${DRV_PATH}" ];
then
    echo "ERROR: No driver path is given!"
fi

if [ -z "${BOARD_PATH}" ];
then
    echo "ERROR: No board path is given!"
fi

# Let's source the board.settings (null.settings before)
BOARD_SETTINGS_FILE=${BOARD_PATH}/board.settings
CFG_DRV_CPU_NAME=
CFG_DRV_EPCS=
CFG_DRV_EPCQ=
CFG_JTAG_CABLE=
CFG_DRV_MAX_HEAP_BYTES=
if [ -f ${BOARD_SETTINGS_FILE} ]; then
    source ${BOARD_SETTINGS_FILE}
else
    echo "ERROR: ${BOARD_SETTINGS_FILE} not found!"
    exit 1
fi

if [ -z "${CFG_DRV_CPU_NAME}" ]; then
    echo "ERROR: The board has no CPU processing drv!"
    exit 1
fi

BSP_PATH=${OUT_PATH}/bsp-${CFG_DRV_CPU_NAME}

BSP_GEN_ARGS="${CFG_DRV_BSP_TYPE} ${BSP_PATH} ${BOARD_PATH}/quartus \
--set hal.enable_c_plus_plus false \
--set hal.enable_clean_exit false \
--set hal.enable_exit false \
--cpu-name ${CFG_DRV_CPU_NAME} \
--set hal.sys_clk_timer ${CFG_DRV_SYS_TIMER_NAME} \
"

if [ -n "${CFG_DRV_TCI_MEM_NAME}" ];
then
    BSP_GEN_ARGS+="--cmd add_section_mapping .tc_i_mem ${CFG_DRV_TCI_MEM_NAME} \
                   --set hal.linker.enable_alt_load_copy_exceptions false "
    echo "INFO: tc_i_mem is used by the system!"
fi

if [ -n "${CFG_DRV_MAX_HEAP_BYTES}" ];
then
    BSP_GEN_ARGS+="--set hal.make.bsp_cflags_user_flags -DALT_MAX_HEAP_BYTES=${CFG_DRV_MAX_HEAP_BYTES} "
    echo "INFO: Specify maximum heap size to ${CFG_DRV_MAX_HEAP_BYTES} BYTES!"
fi

if [ -z "${DEBUG}" ];
then
    BSP_GEN_ARGS+="\
    --set hal.stdout none --set hal.stdin none --set hal.stderr none \
    --set hal.make.bsp_cflags_optimization ${CFG_DRV_BSP_OPT_LEVEL} \
    "
else
    BSP_GEN_ARGS+="--set hal.make.bsp_cflags_optimization -O0"
    echo "INFO: Prepare BSP for debug."
fi

nios2-bsp ${BSP_GEN_ARGS}
RET=$?

if [ ${RET} -ne 0 ]; then
    echo "ERROR: BSP generation returned with error ${RET}!"
    exit ${RET}
fi

# Now fix the generated linker.x file...
chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/fix-linkerx
${OPLK_BASE_DIR}/tools/altera-nios2/fix-linkerx ${BSP_PATH}

# Generate the library fitting to the board
chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/stack.sh
${OPLK_BASE_DIR}/tools/altera-nios2/stack.sh ${BSP_PATH} --out ${OUT_PATH} ${DEBUG_ARG}
RET=$?

if [ ${RET} -ne 0 ]; then
    echo "ERROR: Library generation returned with error ${RET}!"
    exit ${RET}
fi

# Let's source the drivers.settings
CFG_DRV_ARGS=
DRV_SETTINGS_FILE=${DRV_PATH}/drivers.settings
if [ -f ${DRV_SETTINGS_FILE} ]; then
    source ${DRV_SETTINGS_FILE}
else
    echo "ERROR: ${DRV_SETTINGS_FILE} not found!"
    exit 1
fi

if [ -n "${DEBUG}" ];
then
    DRV_OPT_LEVEL=-O0
    DEBUG_MODE=_DEBUG
else
    DEBUG_MODE=NDEBUG
fi

DRV_GEN_ARGS="\
--app-dir ${OUT_PATH} \
--bsp-dir ${BSP_PATH} \
--elf-name ${DRV_NAME}.elf \
--src-files ${DRV_SOURCES} \
--set APP_CFLAGS_OPTIMIZATION ${DRV_OPT_LEVEL} \
--set CFLAGS=${CFLAGS} -D${DEBUG_MODE} \
--set OBJDUMP_INCLUDE_SOURCE 1 \
--set CREATE_OBJDUMP 0 \
--set QSYS_SUB_CPU ${CFG_DRV_PROC_NAME} \
--set QUARTUS_PROJECT_DIR=${BOARD_PATH}/quartus \
${CFG_DRV_ARGS} \
"

# Set the bitstream output path
DRV_GEN_ARGS+="--set QUARTUS_SOF_DIR=\$(QUARTUS_PROJECT_DIR) "

# Get path to board includes
BOARD_INCLUDE_PATH=$(readlink -f "${BOARD_PATH}/include")

if [ ! -d "${BOARD_INCLUDE_PATH}" ];
then
    echo "ERROR: Path to board include does not exist (${BOARD_INCLUDE_PATH})!"
    exit 1
fi

# Add board includes to drv includes
DRV_INCLUDES+=" ${BOARD_INCLUDE_PATH}"

# Add JTAG cable from board.settings
if [ -n "${CFG_JTAG_CABLE}" ];
then
    DRV_GEN_ARGS+="--set DOWNLOAD_CABLE ${CFG_JTAG_CABLE} "
    echo "INFO: Set JTAG Cable to ${CFG_JTAG_CABLE}."
fi

if [ -n "${CFG_DEVICE_ID}" ];
then
    DRV_GEN_ARGS+="--set DOWNLOAD_DEVICE_FLAG=\"--device=${CFG_DEVICE_ID}\" "
    echo "INFO: Set JTAG Chain device Id to ${CFG_DEVICE_ID}."
    export CFG_DEVICE_ID
else
    DRV_GEN_ARGS+="--set DOWNLOAD_DEVICE_FLAG=\"--device=1\" "
fi
# And add stack library
LIB_STACK_DIR=$(find ${OUT_PATH} -type d -name "liboplk*")

DRV_GEN_ARGS+="--use-lib-dir ${LIB_STACK_DIR} "

# And add include to stack proj
LIB_PROJ_DIR=${OPLK_BASE_DIR}/stack/proj/generic/$(basename $LIB_STACK_DIR)
DRV_INCLUDES+=" ${LIB_PROJ_DIR}"

# Add includes
for i in ${DRV_INCLUDES}
do
    DRV_GEN_ARGS+="--inc-dir ${i} "
done

nios2-app-generate-makefile ${DRV_GEN_ARGS}
RET=$?

if [ ${RET} -ne 0 ]; then
    echo "ERROR: Application generation returned with error ${RET}!"
    if [ -n "${CFG_DEVICE_ID}" ];
    then
        unset CFG_DEVICE_ID
    fi
    exit ${RET}
fi

chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/fix-app-makefile
${OPLK_BASE_DIR}/tools/altera-nios2/fix-app-makefile ${OUT_PATH}/Makefile

# Add EPCS flash makefile rules
if [ -n "${CFG_DRV_EPCS}" ]; then
    chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-epcs
    ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-epcs ${OUT_PATH}/Makefile
elif [ -n "${CFG_DRV_EPCQ}" ]; then
    chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-epcq
    ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-epcq ${OUT_PATH}/Makefile
fi

# Add ELF to BIN makefile rules
chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-bin
${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-bin ${OUT_PATH}/Makefile

# Add SOF to RBF makefile rules
chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-rbf
${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-rbf ${OUT_PATH}/Makefile


#TODO: use trap instead of multiple cleanup checks
if [ -n "${CFG_DEVICE_ID}" ];
then
    unset CFG_DEVICE_ID
fi
exit 0
