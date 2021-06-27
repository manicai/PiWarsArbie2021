# Copyright 2021 Ian Glover <ian.glover@gmail.com>
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

import rclpy
from rclpy.node import Node
from arbie_msgs.msg import Gamepad

from . import redboard
from .constants import PadKeys, KeyAction, Channels
from .callbacks import GamepadCallback

SOLENOID_PIN = 20


class Launcher(Node):
    def __init__(self):
        super().__init__('launcher')
        self.subscription = self.create_subscription(
            Gamepad,
            Channels.gamepad,
            self.listener_callback,
            10)
        redboard.output_pin(SOLENOID_PIN)

    @GamepadCallback
    def listener_callback(self, pad_key, key_action):
        if pad_key != PadKeys.button_x:
            return
        if key_action == KeyAction.repeat:
            return

        print("Firing", end=' ')
        if key_action == KeyAction.down:
            # Set solenoid
            redboard.setPin(SOLENOID_PIN, 1)
            print("Trigger pin", SOLENOID_PIN)

        elif key_action == KeyAction.up:
            # Unset solenoid
            redboard.setPin(SOLENOID_PIN, 0)
            print("Release pin", SOLENOID_PIN)


def main(args=None):
    rclpy.init(args=args)

    node = Launcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
