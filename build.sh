#!/bin/bash

function help() {
    echo "$0 [options] [build|upload|all]"
    echo ""
    echo "build      build the code (default action)"
    echo "upload     upload the code"
    echo "all        build and upload if build is success"
    echo ""
    echo "-h         show this help"
    echo "-b         set board (default=esp32:esp32:esp32)"
    echo "-p         set port (default=/dev/ttyESP32)"
}

: ${ARDUINO_BOARD:="esp32:esp32:esp32"}
: ${ARDUINO_PORT:="/dev/ttyESP32"}

board=$ARDUINO_BOARD
port=$ARDUINO_PORT

while getopts "hb:p:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	b)
	    board=$OPTARG
	    ;;
	p)
	    port=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))


target=$1
if [ -z $target ]; then
    target=build
fi

function build() {
    echo "building for $board..."
    arduino-cli compile -b $board .
}

function upload() {
    echo "uploading for $board at $port..."
    arduino-cli upload -b $board -p $port .
    while [ $? -ne 0 ]; do
	arduino-cli upload -b $board -p $port .
    done	
}    

if [ $target == "build" ]; then
    build
fi

if [ $target == "upload" ]; then
    upload
fi

if [ $target == "all" ]; then
    build && upload
fi
