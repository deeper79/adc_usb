obj-m += adc_usb.o

PWD :=$(shell pwd)
Kernelpath :=../../linux_kernel

all:
		make -C /lib/modules/`uname -r`/build M=$(PWD) modules
		
clean:
		make -C $(Kernelpath) M=$(PWD) clean
