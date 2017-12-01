#!/bin/bash

MKSPIFFS=/Users/goranedling/Documents/Arduino/packages/esp8266/tools/mkspiffs/0.2.0/mkspiffs
ESPTOOL=/Users/goranedling/Documents/Arduino/packages/esp8266/tools/esptool/0.4.12/esptool
CURRPATH=$(pwd)
DATA=$1
DATA=/Users/goranedling/Documents/git/nmea2wifi/data

$MKSPIFFS -p 256 -b 8192 -s $((0x3FB000 - 0x100000)) -c $DATA spiffs-image.bin

read -n 1 -s -r -p "Press any key to continue"
echo "Starting upload to device"
echo 
$ESPTOOL -cd nodemcu -ca 0x100000 -cb 115200 -cp /dev/cu.SLAB_USBtoUART -cf spiffs-image.bin
