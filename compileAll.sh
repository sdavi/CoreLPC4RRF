#!/bin/sh


BOARDS=("AZTEEGX5MINI" "SMOOTHIEBOARD" "REARM" "AZSMZ" "MKSSBASE")
NETWORK=("0" "1" "1" "0" "1")


for i in ${!BOARDS[*]}
do
  make distclean
  make firmware BOARD=${BOARDS[$i]} NETWORKING=false OUTPUT_NAME=firmware-${BOARDS[$i]} USE_DFU=false
  if [ "${NETWORK[$i]}" = "1" ]
  then
    make distclean
    make firmware BOARD=${BOARDS[$i]} NETWORKING=true OUTPUT_NAME=firmware-${BOARDS[$i]}-NETWORK USE_DFU=false
  fi

 

done
