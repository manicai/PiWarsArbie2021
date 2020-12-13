#include <libudev.h>
#include <stdio.h>

int main(void)
{
	struct udev *udev_ctx = udev_new();
	struct udev_enumerate *udev_enum = udev_enumerate_new(udev_ctx);

	udev_enumerate_scan_devices(udev_enum);
	struct udev_list_entry *udev_item = udev_enumerate_get_list_entry(udev_enum);
	int count = 1;
	while (udev_item)
	{
		const char* name = udev_list_entry_get_name(udev_item);
		printf("%d %s \n", count++, name);

		struct udev_device *dev = udev_device_new_from_syspath(udev_ctx, name);
		struct udev_list_entry *prop_item = udev_device_get_properties_list_entry(dev);
		while (prop_item)
		{
			const char* property_name = udev_list_entry_get_name(prop_item);
			const char* value = udev_device_get_property_value(dev, property_name);
			printf("    %s :-> %s\n", property_name, value);

			prop_item = udev_list_entry_get_next(prop_item);
		}

		udev_device_unref(dev);
		udev_item = udev_list_entry_get_next(udev_item);
	}

	(void)udev_enumerate_unref(udev_enum);
	(void)udev_unref(udev_ctx);
	return 0;
}
