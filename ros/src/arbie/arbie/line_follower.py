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

import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import redboard

LEFT_SENSOR = 6
CENTRE_SENSOR = 13
RIGHT_SENSOR = 27

SENSORS = [LEFT_SENSOR, CENTRE_SENSOR, RIGHT_SENSOR]


class LineFollowSensor(Node):
    def __init__(self):
        super().__init__('line_follow_sensor')
        self._publisher = self.create_publisher(String, 'line_follow', 10)

    def read_loop(self):
        for sensor in SENSORS:
            redboard.input_pin(sensor)

        while True:
            left = redboard.readPin(LEFT_SENSOR)
            centre = redboard.readPin(CENTRE_SENSOR)
            right = redboard.readPin(RIGHT_SENSOR)

            self.get_logger().info('Line follow sensors : [' +
                                   ', '.join(str(s) for s in [left, centre, right]) +
                                   ']')
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    reader = LineFollowSensor()
    reader.read_loop()
    reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
