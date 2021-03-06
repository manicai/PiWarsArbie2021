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
from arbie_msgs.msg import Gamepad, Motor

from .constants import KeyAction, PadKeys, Channels
from .callbacks import GamepadCallback

# Scale factors to get straight driving.
MOTOR_LEFT_SCALE = 100
MOTOR_RIGHT_SCALE = -100

MIN_VALUE = 50
MAX_VALUE = 100
INCREMENT = 2


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Gamepad,
            Channels.gamepad,
            self.listener_callback,
            10)
        self._publisher = self.create_publisher(Motor, Channels.motors, 10)

        self._multiplier = 0

    @GamepadCallback
    def listener_callback(self, pad_key, key_action):
        # Ignore buttons that aren't part of the gamepad cross.
        print('Controller ', pad_key, key_action)
        if not pad_key.is_cross():
            return

        if key_action == KeyAction.up:
            # Key released - stop
            self._multiplier = 0
        elif key_action == KeyAction.down:
            self._multiplier = MIN_VALUE
        elif key_action == KeyAction.repeat:
            self._multiplier = min(self._multiplier + INCREMENT, MAX_VALUE)

        if pad_key == PadKeys.cross_up:
            motor_left, motor_right = -1, -1
        elif pad_key == PadKeys.cross_down:
            motor_left, motor_right = 1, 1
        elif pad_key == PadKeys.cross_left:
            motor_left, motor_right = -1, 1
        elif pad_key == PadKeys.cross_right:
            motor_left, motor_right = 1, -1
        else:
            assert False, 'Should be unreachable'

        print(motor_left * self._multiplier, motor_right * self._multiplier)
        msg = Motor()
        msg.left_percent = float(motor_left * self._multiplier)
        msg.right_percent = float(motor_right * self._multiplier)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MotorController()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
