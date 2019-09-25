#!/bin/sh

#extract firmware version from header file
VER=`awk 'sub(/.*MAIN_VERSION/,""){print $1}' RepRapFirmware/src/Version.h  | awk 'gsub(/"/, "", $1)'`

OUTPUT=releases/${VER}

mkdir -p releases/${VER}

make distclean
make firmware BUILD=Release MBED=false NETWORKING=true OUTPUT_NAME=firmware USE_DFU=false

if [ -f firmware.bin ]; then
	mv firmware.bin ${OUTPUT}
fi 

