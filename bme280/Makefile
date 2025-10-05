# Makefile for BME280 driver and user-space tools

# Kernel build parameters
ifneq ($(KERNELRELEASE),)
obj-m := bme280_kmod_full.o
bme280_kmod_full-objs := driver_bme.o lib_bme280.o concurrency_kernel_helper.o advanced_bme280.o
ccflags-y := -I$(src)

else
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# User-space compiler
CC := gcc
CFLAGS := -Wall -O2 -I.

# Targets
all: kernel_module user_programs

# Kernel module build
kernel_module:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

# User-space programs
user_programs: concurrency_user bme280_tool main_bme280

concurrency_user: concurrency_user.c
	$(CC) $(CFLAGS) -o concurrency_user concurrency_user.c -luring

bme280_tool: bme280_tool.c
	$(CC) $(CFLAGS) -o bme280_tool bme280_tool.c

main_bme280: main_bme280.c
	$(CC) $(CFLAGS) -o main_bme280 main_bme280.c -luring

# Clean
clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
	rm -f concurrency_user bme280_tool main_bme280
	rm -f *.o *.ko *.mod *.mod.c .*.cmd Module.symvers modules.order
	rm -rf .tmp_versions

# Phony targets
.PHONY: all kernel_module user_programs clean

endif