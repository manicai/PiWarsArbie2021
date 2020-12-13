UDEV_OPTS := $(shell pkg-config --cflags --libs libudev)
CCFLAGS := -Wall -std=c18 -Werror

dump_udev: dump_udev.c
	$(CC) $(CCFLAGS) $< $(UDEV_OPTS) -o $@
