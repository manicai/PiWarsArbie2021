#!/usr/bin/env python
import pyudev as udev


def get_gamepad_devices(ctx: udev.Context = None):
    """Get the device names of connected gamepads."""
    if not ctx:
        ctx = udev.Context()

    # Internal " is needed
    device_name = '"8BitDo Zero 2 gamepad Keyboard"'
    devices = list(ctx.list_devices(subsystem='input',
                                    NAME=device_name))
    prop_name = 'DEVNAME'
    devices = set(child.properties.get(prop_name)
                  for device in devices
                  for child in device.children
                  if child.properties.get(prop_name))
    return devices


def main():
    devices = get_gamepad_devices()
    if devices:
        for device in devices:
            print(device)
    else:
        print('No gamepad connected.')


if __name__ == '__main__':
    main()
