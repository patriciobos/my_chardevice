
obj-m += my_chardevice.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	#gcc test_module.c -o test_module
	gcc writer.c -o writer
	gcc reader.c -o reader


clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
#	·rm test_module
	rm writer
	rm reader
