KERNELDIR := /home/dar/Project/linux/kernal_source/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/
CURRENT_PATH := $(shell pwd)

obj-m := mpu6050.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
	date
	$(CC) mpu6050Demo.c -o mpu6050Demo -Wall -pthread -O2

install:
	scp mpu6050Demo mpu6050.ko root@192.168.192.5:/home/root

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	rm mpu6050Demo -rf