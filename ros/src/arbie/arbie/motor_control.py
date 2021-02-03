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

from .constants import KeyAction, PadKeys


# Scale factors to get straight driving.
MOTOR_LEFT_SCALE = 100
MOTOR_RIGHT_SCALE = -100


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            String,
            'controller',
            self.listener_callback,
            10)
        self._publisher = self.create_publisher(String, 'motor', 10)

    def listener_callback(self, msg):
        btn_text, action_text = [s.strip() for s in msg.data.split(' : ')]
        btn, action = PadKeys[btn_text], KeyAction[action_text]

        # Ignore buttons that aren't part of the gamepad cross.
        if not btn.is_cross():
            return
        # Ignore repeat
        if action == KeyAction.repeat:
            return

        if action == KeyAction.up:
            # Key released - stop
            motor_left, motor_right = 0, 0
        elif btn == PadKeys.cross_up:
            motor_left, motor_right = 1, 1
        elif btn == PadKeys.cross_down:
            motor_left, motor_right = -1, -1
        elif btn == PadKeys.cross_left:
            motor_left, motor_right = -1, 1
        elif btn == PadKeys.cross_right:
            motor_left, motor_right = 1, -1
        else:
            assert False, 'Should be unreachable'

        message = str(motor_left) + ' : ' + str(motor_right)
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MotorController()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
