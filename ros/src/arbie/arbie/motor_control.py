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
from . import redboard


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
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        btn_text, action_text = [s.strip() for s in msg.data.split(' : ')]
        btn, action = PadKeys[btn_text], KeyAction[action_text]
        self.get_logger().info('I heard: "%s - %s"' % (btn, action))

        # Ignore buttons that aren't part of the gamepad cross.
        if not btn.is_cross():
            return
        # Ignore repeat
        if action == KeyAction.repeat:
            return

        self.get_logger().info('Cross action')
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

        motor_left *= MOTOR_LEFT_SCALE
        motor_right *= MOTOR_RIGHT_SCALE

        self.get_logger().info('Motors : left %d, right %d' % (motor_left, motor_right))

        redboard.M1(motor_left)
        redboard.M2(motor_right)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MotorController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
