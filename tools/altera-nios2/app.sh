#!/bin/bash
################################################################################
#
# App generator script for Altera Nios II
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

APP_PATH=$1
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
            echo "$ app.sh [APP] [BOARD] [OPTIONS]"
            echo "APP       ... Path to application project (app.settings)"
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

if [ -z "${APP_PATH}" ];
then
    echo "ERROR: No application path is given!"
fi

if [ -z "${BOARD_PATH}" ];
then
    echo "ERROR: No board path is given!"
fi

# Let's source the app.settings
APP_TYPE=
APP_CFLAGS=
APP_SETTINGS_FILE=${APP_PATH}/app.settings
if [ -f ${APP_SETTINGS_FILE} ]; then
    source ${APP_SETTINGS_FILE}
else
    echo "ERROR: ${APP_SETTINGS_FILE} not found!"
    exit 1
fi

# Let's source the board.settings (null.settings before)
BSP_CFLAGS=
BOARD_SETTINGS_FILE=${BOARD_PATH}/board.settings
if [ -f ${BOARD_SETTINGS_FILE} ]; then
    source ${BOARD_SETTINGS_FILE}
else
    echo "ERROR: ${BOARD_SETTINGS_FILE} not found!"
    exit 1
fi

# Decide if it is driver or app
case ${APP_TYPE} in
    app)
        echo "INFO: Generate application for app."
        SEL_SUB_NAME=${CFG_APP_SUB_NAME}
        SEL_PROC_NAME=${CFG_APP_PROC_NAME}
        SEL_CPU_NAME=${CFG_APP_CPU_NAME}
        SEL_SYS_TIMER_NAME=${CFG_APP_SYS_TIMER_NAME}
        SEL_TCI_MEM_NAME=${CFG_APP_TCI_MEM_NAME}
        SEL_BSP_TYPE=${CFG_APP_BSP_TYPE}
        SEL_BSP_OPT_LEVEL=${CFG_APP_BSP_OPT_LEVEL}
        SEL_MAX_HEAP_BYTES=${CFG_APP_MAX_HEAP_BYTES}
        SEL_EPCS=${CFG_APP_EPCS}
        SEL_DEF_MEM_NAME=${CFG_APP_DEF_MEM_NAME}
        SEL_HOSTED_BOOT=${CFG_APP_HOSTED_BOOT}
        ;;
    drv)
        echo "INFO: Generate application for driver."
        SEL_SUB_NAME=${CFG_DRV_SUB_NAME}
        SEL_PROC_NAME=${CFG_DRV_PROC_NAME}
        SEL_CPU_NAME=${CFG_DRV_CPU_NAME}
        SEL_SYS_TIMER_NAME=${CFG_DRV_SYS_TIMER_NAME}
        SEL_TCI_MEM_NAME=${CFG_DRV_TCI_MEM_NAME}
        SEL_BSP_TYPE=${CFG_DRV_BSP_TYPE}
        SEL_BSP_OPT_LEVEL=${CFG_DRV_BSP_OPT_LEVEL}
        SEL_MAX_HEAP_BYTES=${CFG_DRV_MAX_HEAP_BYTES}
        SEL_EPCS=${CFG_DRV_EPCS}
        SEL_DEF_MEM_NAME=${CFG_DRV_DEF_MEM_NAME}
        SEL_HOSTED_BOOT=${CFG_DRV_HOSTED_BOOT}
        ;;
    *)
        echo "ERROR: No APP_TYPE specified in ${APP_SETTINGS_FILE}!"
        echo "       Valid values are app or drv!"
    exit 1
        ;;
esac

if [ -z "${SEL_CPU_NAME}" ]; then
    echo "ERROR: The board has no CPU processing ${APP_TYPE}!"
    exit 1
fi

BSP_PATH=${OUT_PATH}/bsp-${SEL_CPU_NAME}

BSP_GEN_ARGS="${SEL_BSP_TYPE} ${BSP_PATH} ${BOARD_PATH}/quartus \
--set hal.enable_c_plus_plus false \
--set hal.enable_clean_exit false \
--set hal.enable_exit false \
--set hal.enable_reduced_device_drivers true \
--set hal.enable_small_c_library false \
--set hal.enable_lightweight_device_driver_api true \
--cpu-name ${SEL_CPU_NAME} \
--set hal.sys_clk_timer ${SEL_SYS_TIMER_NAME} \
"

if [ -n "${SEL_TCI_MEM_NAME}" ];
then
    BSP_GEN_ARGS+="--cmd add_section_mapping .tc_i_mem ${SEL_TCI_MEM_NAME} \
                   --set hal.linker.enable_alt_load_copy_exceptions false "
    echo "INFO: tc_i_mem is used by the system!"
fi

# Add flag for explicitly using EPCS flash.
if [ -n "${SEL_EPCS}" ]; then
    BSP_CFLAGS+="-DALT_USE_EPCS_FLASH "
fi

if [ -n "${SEL_DEF_MEM_NAME}" ];
then
    BSP_GEN_ARGS+="--default_sections_mapping ${SEL_DEF_MEM_NAME} "
    echo "INFO: The default memory is changed to ${SEL_DEF_MEM_NAME}"
fi

if [ -n "${SEL_MAX_HEAP_BYTES}" ];
then
    BSP_CFLAGS+="-DALT_MAX_HEAP_BYTES=${SEL_MAX_HEAP_BYTES} "
    echo "INFO: Specify maximum heap size to ${SEL_MAX_HEAP_BYTES} BYTES!"
fi

if [ -z "${DEBUG}" ];
then
    BSP_GEN_ARGS+="\
    --set hal.stdout none --set hal.stdin none --set hal.stderr none \
    --set hal.make.bsp_cflags_optimization ${SEL_BSP_OPT_LEVEL} \
    "
else
    BSP_GEN_ARGS+="--set hal.make.bsp_cflags_optimization -O0 "
    echo "INFO: Prepare BSP for debug."
fi

if [ -n "${BSP_CFLAGS}" ];
then
    BSP_GEN_ARGS+="--set hal.make.bsp_cflags_user_flags ${BSP_CFLAGS} "
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

# The stack.sh script has created a temporary file including the just created
# library name!
TMP_LIB_FILE=${OUT_PATH}/created-lib.tmp

if [ -f "${TMP_LIB_FILE}" ]; then
    echo "INFO: Link lib${LIB_NAME} to application."
    LIB_NAME=$(cat ${TMP_LIB_FILE})
    rm -f ${TMP_LIB_FILE}
else
    echo "WARNING: The stack.sh script has not generated the ${TMP_LIB_FILE} file!"
    echo "         Thus, I must guess what library shall be linked to the application!"
    LIB_NAME=
fi

if [ -n "${DEBUG}" ];
then
    APP_OPT_LEVEL=-O0
    DEBUG_MODE=_DEBUG
else
    DEBUG_MODE=NDEBUG
fi

APP_GEN_ARGS="\
--app-dir ${OUT_PATH} \
--bsp-dir ${BSP_PATH} \
--elf-name ${APP_NAME}.elf \
--src-files ${APP_SOURCES} \
--set APP_CFLAGS_OPTIMIZATION ${APP_OPT_LEVEL} \
--set CFLAGS=${CFLAGS} -D${DEBUG_MODE} \
--set OBJDUMP_INCLUDE_SOURCE 1 \
--set CREATE_OBJDUMP 0 \
--set QSYS_SUB_CPU ${SEL_PROC_NAME} \
--set QUARTUS_PROJECT_DIR=${BOARD_PATH}/quartus \
--set OPLK_BASE_DIR=${OPLK_BASE_DIR} \
"

# Add flag for explicitly using EPCS flash.
if [ -n "${SEL_EPCS}" ]; then
    APP_CFLAGS+="-DALT_USE_EPCS_FLASH "
fi

# Add application CFLAGS
if [ -n "${APP_CFLAGS}" ]; then
    APP_GEN_ARGS+="--set APP_CFLAGS_DEFINED_SYMBOLS=${APP_CFLAGS} "
fi

# Get path to board includes
BOARD_INCLUDE_PATH=$(readlink -f "${BOARD_PATH}/include")

if [ ! -d "${BOARD_INCLUDE_PATH}" ];
then
    echo "ERROR: Path to board include does not exist (${BOARD_INCLUDE_PATH})!"
    exit 1
fi

# Add board includes to app includes
APP_INCLUDES+=" ${BOARD_INCLUDE_PATH}"

# Add JTAG cable from board.settings
if [ -n "${CFG_JTAG_CABLE}" ];
then
    APP_GEN_ARGS+="--set DOWNLOAD_CABLE=${CFG_JTAG_CABLE} "
    echo "INFO: Set JTAG Cable to ${CFG_JTAG_CABLE}."
fi

if [ -n "${CFG_DEVICE_ID}" ];
then
    APP_GEN_ARGS+="--set DOWNLOAD_DEVICE_FLAG=\"--device=${CFG_DEVICE_ID}\" "
    echo "INFO: Set JTAG Chain device Id to ${CFG_DEVICE_ID}."
    export CFG_DEVICE_ID
fi

# And add stack library
if [ -n "${LIB_NAME}" ]; then
    LIB_STACK_DIR=$(find ${OUT_PATH} -type d -name "lib${LIB_NAME}")
else
LIB_STACK_DIR=$(find ${OUT_PATH} -type d -name "liboplk*")
fi

APP_GEN_ARGS+="--use-lib-dir ${LIB_STACK_DIR} "

# And add include to stack proj
LIB_PROJ_DIR=${OPLK_BASE_DIR}/stack/proj/generic/$(basename $LIB_STACK_DIR)
APP_INCLUDES+=" ${LIB_PROJ_DIR}"

# Add includes
for i in ${APP_INCLUDES}
do
    APP_GEN_ARGS+="--inc-dir ${i} "
done

nios2-app-generate-makefile ${APP_GEN_ARGS}
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
if [ -n "${SEL_EPCS}" ]; then
    # Create system.mk to get system info
    SOPCINFO_FILE=$(find ${BOARD_PATH}/quartus -name *.sopcinfo)
    SWINFO_FILE=${OUT_PATH}/swinfo
    SETTINGS_FILE=${OUT_PATH}/system.mk

    sopcinfo2swinfo --input=${SOPCINFO_FILE} --output=${SWINFO_FILE}
    RET=$?

    if [ ${RET} -ne 0 ]; then
        echo "ERROR: Generating ${SWINFO_FILE} file failed with error ${RET}!"
        exit ${RET}
    fi

    swinfo2header --swinfo ${SWINFO_FILE} --format mk --single ${SETTINGS_FILE} --module ${SEL_CPU_NAME}
    RET=$?

    if [ ${RET} -ne 0 ]; then
        echo "ERROR: Generating ${SETTINGS_FILE} file failed with error ${RET}!"
        exit ${RET}
    fi

    # Remove unused swinfo file
    rm -f ${SWINFO_FILE}

    chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-epcs
    ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-epcs ${OUT_PATH}/Makefile
fi

if [ -n "${SEL_HOSTED_BOOT}" ]; then
    # Add C5 SoC hosted boot makefile rules
    chmod +x ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-c5socboot
    ${OPLK_BASE_DIR}/tools/altera-nios2/add-app-makefile-c5socboot ${OUT_PATH}/Makefile
fi

#TODO: use trap instead of multiple cleanup checks
if [ -n "${CFG_DEVICE_ID}" ];
then
    unset CFG_DEVICE_ID
fi

exit 0
