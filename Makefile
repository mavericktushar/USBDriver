obj-m := usb_skeleton.o

KDIR := /lib/modules/$(shell uname -r)/build

all:
		$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
		$(MAKE) -C $(KDIR) M=$(PWD) clean
		$(RM) Module.markers modules.order