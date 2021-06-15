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
from arbie_msgs.msg import Gamepad, LineSensor, Motor

from .constants import KeyAction, PadKeys, Channels
from .callbacks import GamepadCallback, LineSensorCallback

MOTOR_SPEED = 60


class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')
        self.active = False
        self.gamepad_subscription = self.create_subscription(
            Gamepad,
            Channels.gamepad,
            self.gamepad_callback,
            10)
        self.line_subscription = self.create_subscription(
            LineSensor,
            Channels.line_follow_sensor,
            self.line_follow_callback,
            10)

        self._publisher = self.create_publisher(Motor, Channels.motors, 10)

    @GamepadCallback
    def gamepad_callback(self, pad_key, key_action):
        if pad_key == PadKeys.right_shoulder and key_action == KeyAction.up:
            self.active = True
            self.get_logger().info('Line control : activated')
        if pad_key == PadKeys.left_shoulder and key_action == KeyAction.up:
            self.active = False
            self.get_logger().info('Line control : deactivated')

    @LineSensorCallback
    def line_follow_callback(self, left, centre, right):
        if not self.active:
            return

        if left and not right:
            self.get_logger().info('Line control : turn right')
            motor_left = 0
            motor_right = MOTOR_SPEED
        elif right and not left:
            self.get_logger().info('Line control : turn left')
            motor_left = MOTOR_SPEED
            motor_right = 0
        elif centre:
            self.get_logger().info('Line control : straight ahead')
            motor_left = MOTOR_SPEED
            motor_right = MOTOR_SPEED
        else:
            self.get_logger().info('Line control : confused!')
            motor_left = 0
            motor_right = 0

        msg = Motor()
        msg.left_percent = motor_left
        msg.right_percent = motor_right
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
