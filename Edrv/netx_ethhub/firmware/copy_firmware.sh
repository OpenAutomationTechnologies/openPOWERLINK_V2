#! /bin/sh

# The script copies the firmware files for the xc units to the appropriate place.

DESTDIR=/usr/local/lib/firmware

mkdir -p ${DESTDIR}
cp eth_2port_hub_xc0.bin ${DESTDIR}/xc0.bin
cp eth_2port_hub_xc1.bin ${DESTDIR}/xc1.bin

