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

from .constants import KeyAction, PadKeys, Channels


MOTOR_SPEED = 60


class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')
        self.active = False
        self.gamepad_subscription = self.create_subscription(
            String,
            Channels.gamepad,
            self.gamepad_callback,
            10)
        self.line_subscription = self.create_subscriptino(
            String,
            Channels.line_follow_sensor,
            self.line_follow_callback,
            10)

        self._publisher = self.create_publisher(String, 'motor', 10)

    def gamepad_callback(self, msg):
        btn_text, action_text = [s.strip() for s in msg.data.split(' : ')]
        btn, action = PadKeys[btn_text], KeyAction[action_text]

        if btn == PadKeys.right_shoulder and action == KeyAction.up:
            self.active = True
            self.get_logger().info('Line control : activated')
        if btn == PadKeys.left_shoulder and action == KeyAction.up:
            self.active = False
            self.get_logger().info('Line control : deactivated')

    def line_follow_callback(self, msg):
        if not self.active:
            return

        left, centre, right = (int(x) for x in msg.data.split(','))

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

        message = str(motor_left) + ' : ' + str(motor_right)
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = LineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
