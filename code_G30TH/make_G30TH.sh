#!/bin/bash
# make_G30TH.sh

export BOARD="G30TH"
echo Building for $BOARD


if [ $BOARD ]
then
    cd /home/john/micropython/ports/stm32
    if [ "$1" == "--clean" ]
    then
        make BOARD=$BOARD clean
        exit
    fi
    if make -j 2 V=1 BOARD=$BOARD
    then
        sleep 1
        make V=1 BOARD=$BOARD deploy
        cd -
#        sleep 1
#        rshell
    else
        echo Build failure
    fi
else
    echo "Wrong board type."
fi
