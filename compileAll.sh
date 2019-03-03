#!/bin/sh

#make distclean
#make firmware BUILD=Release MBED=false NETWORKING=false OUTPUT_NAME=firmware USE_DFU=false
make distclean
make firmware BUILD=Release MBED=false NETWORKING=true OUTPUT_NAME=firmware-NETWORK USE_DFU=false


 

