About
-----

The HC-SR04 is an ultrasonic distance sensor to be attached on 2 5V GPIO pins. 
This driver uses the interrupt logic of the GPIO driver to measure the 
distance in a non-blocking, precise and load-independend way. Unlike 
user land measurement methods you can even compile a linux kernel 
while doing measurements without getting weird results.

The driver has been tested using a Raspberry Pi 1, but in theory should
work on any device that supports GPIO hardware. 

Keep in mind that the HC-SR04 is a 5 volts device so attatching it directly
to the raspberry (3.3 Volts) pins is probably not a good idea. There
are tutorials on the net explaining how to solder 2 resistors such that
it works with the 3.3 Volts pins.

Building
--------

You have two options to compile this driver for the Raspberry Pi:

- either compile it on a linux host using a cross development environment
   (recommended)
- or directly on the raspberry

To compile it using a cross development environment you'll need to install
a cross development environment first (ARM cross compiler) and then obtain
the raspberry pi kernel sources (these are different from the vanilla 
kernel), see https://www.raspberrypi.org/documentation/linux/kernel
for instructions. Once the kernel is compiled and installed on the 
raspberry edit the Makefile of this repo so it says something like:

```  ARCH=arm
  CROSS_COMPILE=$(HOME)/raspberry/cross-dev/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
  KERNEL_DIR=$(HOME)/raspberry/linux
```

Type 

```
  make 
```

and keep your fingers crossed ;) The result should be a file named hc-sro4.ko
which is the linux kernel module to be inserted using:

```
  insmod hc-sro4.ko
```

on the raspberry. 

Note that the kernel version running on the raspberry *must* match the 
version the module is built against, else insmod will not work.

Comiling on the raspberry is not recommended, since this requires either
a kernel compile (takes hours) or using the kernel dev headers from the
raspbian repo (which exists only for certain old kernels - this seems not
to be supported).

If you need help compiling you may want to drop me a message, I'll be 
happy to help out.

Using the driver
----------------

This driver uses IIO software triggers to interface with userland.

To configure a device do a

```
   mkdir /config/iio/triggers/hc-sr04/sensor0
```

(you need to mount configfs to /config first)

Then configure the ECHO and TRIG pins (this also accepts symbolic names
configured in the device tree)

```
   echo 23 > /config/iio/triggers/hc-sr04/sensor0/trig_pin
   echo 24 > /config/iio/triggers/hc-sr04/sensor0/echo_pin
```

Then you can measure distance with:

```
   cat /sys/devices/trigger0/measure
```

(trigger0 is the device name as reported by
 /config/iio/triggers/hc-sr04/sensor0/dev_name

To convert to centimeters, multiply by 17150 and divide by 1000000 (air)

That's all.

Enjoy and please Star this repo if you like it.

- Johannes


