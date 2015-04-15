#!/bin/bash
# $ program-epcs.sh [SOF_FILE] [ELF_FILE] [OPTIONS]

echo
echo "*** progam-epcs.sh ***"

SOF_FILE=$1
ELF_FILE=$2
shift 2

PROG_PATH=
CABLE_NAME=
NOPROG=
while [ $# -gt 0 ]
do
    case "$1" in
        --prog)
            shift
            PROG_PATH=$1
            ;;
        --cable)
            shift
            CABLE_NAME=$1
            ;;
        --noprog)
            NOPROG=1
            ;;
        --help)
            echo "$ program-epcs.sh [SOF_FILE] [ELF_FILE] [OPTIONS]"
            echo " [SOF_FILE] ............. SOF file"
            echo " [ELF_FILE] ............. ELF file"
            echo "OPTIONS:"
            echo " --prog [PATH] .......... Path to *.cdf and *.cof files."
            echo "                          If not given, current path is searched."
            echo " --cable [CABLE_NAME] ... Specify JTAG cable name to be used."
            echo "                          If not given, uses default cable."
            echo " --noprog ............... Do no start programming, only generate files."
            exit 1
            ;;
        *)
            ;;
    esac
    shift
done

if [ -z "${PROG_PATH}" ];
then
    PROG_PATH=.
fi

# find for quartus_cpf and quartus_pgm file
echo
echo "INFO: Search for cof and cfg files in path ${PROG_PATH}"
CPF_FILE=$(find ${PROG_PATH} -type f -name '*.cof')
PGM_FILE=$(find ${PROG_PATH} -type f -name '*.cdf')

if [ -z "${CPF_FILE}" ];
then
    echo
    echo "ERROR: No *.cof found in ${PROG_PATH}!"
    exit 1
fi

if [ -z "${PGM_FILE}" ];
then
    echo
    echo "ERROR: No *.cdf found in ${PROG_PATH}!"
    exit 1
fi

HW_FILE=hw
SW_FILE=sw
IMG_FILE=image

# Create flash files...
echo
echo "INFO: Create flash file for bitstream ${SOF_FILE} ..."
CMD="sof2flash --epcs --input=${SOF_FILE} --output=${HW_FILE}.flash --compress"
${CMD} || {
    echo
    echo "ERROR: SOF2FLASH failed!"
    exit 1
}

echo
echo "INFO: Create flash file for elf ${ELF_FILE} ..."
CMD="elf2flash --epcs --input=${ELF_FILE} --outfile=${SW_FILE}.flash \
--after=${HW_FILE}.flash \
"
${CMD} || {
    echo
    echo "ERROR: ELF2FLASH failed!"
    exit 1
}

# Concatenate hard- and software srecs
echo
echo "INFO: Concatenate bitstream and elf to a single flash file..."
cat ${HW_FILE}.flash ${SW_FILE}.flash > ${IMG_FILE}.flash

# Remove single srecs
rm -rf ${HW_FILE}.flash ${SW_FILE}.flash

# Create intel hex out of flash file
echo
echo "INFO: Convert flash file to intel hex format..."
CMD="nios2-elf-objcopy -I srec -O ihex ${IMG_FILE}.flash ${IMG_FILE}.hex"
${CMD} || {
    echo
    echo "ERROR: OBJCOPY hardware srec to hex failed!"
    exit 1
}

# Convert hex file into jic file
echo
echo "INFO: Convert intel hex to jic file with Quartus convert tool..."
CMD="quartus_cpf -c ${CPF_FILE}"
${CMD} || {
    echo
    echo "ERROR: QUARTUS CONVERT of hex to jic failed!"
    exit 1
}

# Check if noprog is set
if [ -n "${NOPROG}" ];
then
    echo
    echo "INFO: Skip programming EPCS."
    exit 0
fi

# Program EPCS with jic file
CMD="quartus_pgm ${PGM_FILE}"

if [ -n "${CABLE_NAME}" ];
then
    echo
    echo "INFO: Use cable ${CABLE_NAME} to program EPCS."
    CMD="${CMD} \"-c ${CABLE_NAME}\" "
fi

echo
echo "INFO: Program EPCS with generated jic file..."
${CMD} || {
    echo
    echo "ERROR: QUARTUS PROGRAM failed!"
    exit 1
}

echo
echo "INFO: Programming done! Reset your hardware platform."

exit 0
