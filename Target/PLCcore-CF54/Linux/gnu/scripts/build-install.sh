#!/bin/sh

#***************************************************************************#
#                                                                           #
#  SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29              #
#  www.systec-electronic.com                                                #
#                                                                           #
#  File:         build-install.sh                                           #
#  Description:  install builder script for openPOWERLINK firmware          #
#                                                                           #
#  -----------------------------------------------------------------------  #
#                                                                           #
#  Revision History:                                                        #
#                                                                           #
#  2007/11/05 d.k.:     Initial version                                     #
#  2008/10/07 d.k.:     Adaption for openPOWERLINK on ECUcore-5484          #
#                                                                           #
#****************************************************************************

if [ -z "${INST_FILE}" ] ; then
  INST_FILE=openPOWERLINK_demo_CN.sh
fi

TOPDIR=`pwd`
INSTALLDIR=./
TARGETDIR=${INSTALLDIR}target/
SCRIPTDIR=`dirname $0`
INST_FILE=${INSTALLDIR}${INST_FILE}
SFX_HEADER=${SCRIPTDIR}/sfx-header
TMP_TAR_BALL=openPOWERLINK_demo.tar.gz

ETCDIR=${TARGETDIR}etc/
BINDIR=${TARGETDIR}bin/


echo "Prepare installation target directory..."

rm -f -R $TARGETDIR

mkdir -p ${ETCDIR}

dos2unix < ${SCRIPTDIR}/rc.usr > ${ETCDIR}rc.usr
chmod  +x  ${ETCDIR}rc.usr

dos2unix < ${SCRIPTDIR}/autostart > ${ETCDIR}autostart
chmod  +x  ${ETCDIR}autostart

mkdir -p $BINDIR

${M68K_CC_PREFIX}strip --strip-unneeded -o ${BINDIR}cf54drv.ko ${INSTALLDIR}cf54drv.ko

${M68K_CC_PREFIX}strip --strip-unneeded -o ${BINDIR}epl.ko ${INSTALLDIR}epl.ko


dos2unix < ${SCRIPTDIR}/install.sh > ${TARGETDIR}install.sh
chmod +x ${TARGETDIR}install.sh




cd $TARGETDIR
echo "Build tarball with install files..."
tar -czvf ../$TMP_TAR_BALL ./*
cd -

dos2unix $SFX_HEADER

echo "Build install file '$INST_FILE'..."
echo cat $SFX_HEADER $TMP_TAR_BALL ">" $INST_FILE
cat $SFX_HEADER ${INSTALLDIR}$TMP_TAR_BALL > $INST_FILE

chmod +x $INST_FILE

echo "done."

