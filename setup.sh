#!/bin/bash

if [ $UID != 0 ] ; then
	echo "Please run as root."
	exit 1
fi
insmod hc-sro4.ko 
mount config /config/ -t configfs
mkdir /config/iio/triggers/hc-sro4/sensor0
echo 23 > /config/iio/triggers/hc-sro4/sensor0/trig_pin  
echo 24 > /config/iio/triggers/hc-sro4/sensor0/echo_pin  
