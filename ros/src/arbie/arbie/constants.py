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

import enum


@enum.unique
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

    def __str__(self):
        return self.name

    def is_cross(self):
        return self.value in [self.cross_left.value, self.cross_up.value,
                              self.cross_right.value, self.cross_down.value]


@enum.unique
class KeyAction(enum.Enum):
    down = 1
    repeat = 2
    up = 0

    def __str__(self):
        return self.name
