#!/bin/bash

MKSPIFFS=/Users/goranedling/Documents/Arduino/packages/esp8266/tools/mkspiffs/0.2.0/mkspiffs
ESPTOOL=/Users/goranedling/Documents/Arduino/packages/esp8266/tools/esptool/0.4.12/esptool
ESPOTA=/Users/goranedling/Documents/Arduino/packages/esp8266/hardware/esp8266/2.4.0/tools/espota.py
CURRPATH=$(pwd)
DATA=$1
DATA=/Users/goranedling/Documents/git/nmea2wifi/data

## 4M3 
##$MKSPIFFS -p 256 -b 8192 -s $((0x3FB000 - 0x100000)) -c $DATA spiffs-image.bin

##16M15
$MKSPIFFS -p 256 -b 8192 -s $((0xFFB000 - 0x100000)) -c $DATA spiffs-image.bin

read -n 1 -s -r -p "Press any key to continue"
echo "Starting upload to device"
echo 
$ESPTOOL -cd nodemcu -ca 0x100000 -cb 115200 -cp /dev/cu.SLAB_USBtoUART -cf spiffs-image.bin

## -s for SPIFFS
#python $ESPOTA -r -i 192.168.1.212 -p 8266 --auth= -f /Users/goranedling/Documents/Arduino/build/nmea2wifi/nmea2wifi.bin
