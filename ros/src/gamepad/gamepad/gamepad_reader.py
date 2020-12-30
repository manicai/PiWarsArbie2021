# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import enum
import struct
import time

import pyudev as udev

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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


class GamepadReader(Node):
    def __init__(self):
        super().__init__('gamepad_reader')
        self.publisher_ = self.create_publisher(String, 'controller', 10)

    def read_loop(self):
        while True:
            # Try to get open the reader input. If it
            # doesn't exist then go to sleep for a while
            # before trying again.
            devices = get_gamepad_devices()
            if not devices:
                time.sleep(10)
                continue

            # Assume only one controller.
            dev_path = devices.pop()

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
                        msg = String()
                        msg.data = "%s : %s" % (PadKeys(code), KeyAction(value))
                        self.publisher_.publish(msg)
                        self.get_logger().info('Gamepad input: ' + msg.data)

                    event = dev_file.read(struct_input_event_size)
            finally:
                dev_file.close()


def main(args=None):
    rclpy.init(args=args)

    reader = GamepadReader()
    reader.read_loop()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
