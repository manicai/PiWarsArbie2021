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
from std_msgs.msg import String

from . import redboard
from .constants import PadKeys, KeyAction, Channels

# Scale factors to get straight driving.
MOTOR_LEFT_SCALE = 100
MOTOR_RIGHT_SCALE = -100


class Launcher(Node):
    def __init__(self):
        super().__init__('launcher')
        self.subscription = self.create_subscription(
            String,
            Channels.gamepad,
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        btn_text, action_text = [s.strip() for s in msg.data.split(' : ')]
        btn, action = PadKeys[btn_text], KeyAction[action_text]

        if btn != PadKeys.button_x:
            return

        if action == KeyAction.down:
            # Set solenoid
            pass
        elif action == KeyAction.up:
            # Unset solenoid
            pass


def main(args=None):
    rclpy.init(args=args)

    node = Launcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
