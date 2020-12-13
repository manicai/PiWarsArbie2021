#!/usr/bin/env python
import pyudev as udev
import enum
import struct


def get_gamepad_devices(ctx: udev.Context = None):
    """Get the device names of connected gamepads."""
    if not ctx:
        ctx = udev.Context()

    # Internal " is needed for match to succeed.
    device_name = '"8BitDo Zero 2 gamepad Keyboard"'
    devices = list(ctx.list_devices(subsystem='input',
                                    NAME=device_name))
    prop_name = 'DEVNAME'
    devices = set(child.properties.get(prop_name)
                  for device in devices
                  for child in device.children
                  if child.properties.get(prop_name))
    return devices


class PadKeys(enum.Enum):
    """Input codes for gamepad keys."""
    # Experimentally determined.
    left_shoulder = 37
    right_shoulder = 50
    cross_up = 46
    cross_left = 18
    cross_right = 33
    cross_down = 32
    button_select = 49
    button_start = 24
    button_x = 35
    button_y = 23
    button_a = 34
    button_b = 36


class KeyAction(enum.Enum):
    down = 1
    repeat = 2
    up = 0


def read_device(dev_path: str):
    # From /usr/include/linux/input.h
    struct_input_event_format = 'llHHI'
    struct_input_event_size = struct.calcsize(struct_input_event_format)

    # From /usr/include/linux/input-event-codes.h
    ev_key = 0x01
    dev_file = open(dev_path, "rb")
    try:
        event = dev_file.read(struct_input_event_size)
        while event:
            (_, _, evt_type, code, value) = struct.unpack(struct_input_event_format, event)
            if evt_type == ev_key:
                print("%s : %s" % (PadKeys(code), KeyAction(value)))

            event = dev_file.read(struct_input_event_size)
    finally:
        dev_file.close()


def main():
    devices = get_gamepad_devices()
    if devices:
        for device in devices:
            print(device)
    else:
        print('No gamepad connected.')
        return

    read_device(devices.pop())


if __name__ == '__main__':
    main()
