obj-m += distance.o

KERNEL_DIR=$(HOME)/raspberry/build
CC=arm-linux-gnueabihf-gcc

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
