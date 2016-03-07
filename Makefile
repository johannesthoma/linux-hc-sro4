obj-m += distance.o

# KERNEL_DIR=/lib/modules/3.18.0-trunk-rpi/build
KERNEL_DIR=../linux
# CC=arm-linux-gnueabihf-gcc

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
