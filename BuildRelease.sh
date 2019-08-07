#!/bin/sh

make distclean
make firmware BUILD=Release MBED=false NETWORKING=true OUTPUT_NAME=firmware USE_DFU=false


 

