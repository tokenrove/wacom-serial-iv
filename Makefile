obj-m += wacom_serial.o

all: modules inputattach

modules:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

test:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

.PHONY: all test clean
