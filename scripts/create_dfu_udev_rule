#!/bin/bash
#

if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root"
   exit
fi

NAME=99-arduino-101.rules

echo >/etc/udev/rules.d/$NAME
echo \# Arduino 101 in DFU Mode >>/etc/udev/rules.d/$NAME
echo SUBSYSTEM==\"tty\", ENV{ID_REVISION}==\"8087\", ENV{ID_MODEL_ID}==\"0ab6\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\", ENV{ID_MM_CANDIDATE}=\"0\" >>/etc/udev/rules.d/$NAME
echo SUBSYSTEM==\"usb\", ATTR{idVendor}==\"8087\", ATTR{idProduct}==\"0aba\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\" >>/etc/udev/rules.d/$NAME

udevadm control --reload-rules
udevadm trigger
