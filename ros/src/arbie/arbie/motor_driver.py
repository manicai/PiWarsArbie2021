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
from arbie_msgs.msg import Motor

from . import redboard
from .constants import Channels
from .callbacks import MotorCallback

# Scale factors to get straight driving.
MOTOR_LEFT_SCALE = 100
MOTOR_RIGHT_SCALE = -100


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            Motor,
            Channels.motors,
            self.listener_callback,
            10)

    @MotorCallback
    def listener_callback(self, left_percent, right_percent):
        motor_left = left_percent / 100.0 * MOTOR_LEFT_SCALE
        motor_right = right_percent / 100.0 * MOTOR_RIGHT_SCALE

        self.get_logger().info('Motors : left %d, right %d' % (motor_left, motor_right))

        redboard.M1(motor_left)
        redboard.M2(motor_right)


def main(args=None):
    rclpy.init(args=args)

    driver = MotorDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
