from .constants import PadKeys, KeyAction


def GamepadCallback(func):
    def wrapper(self, msg):
        pad_key = PadKeys(msg.pad_key)
        key_action = KeyAction(msg.key_action)
        return func(self, pad_key, key_action)

    return wrapper


def LineSensorCallback(func):
    def wrapper(self, msg):
        return func(self, msg.left, msg.center, msg.right)

    return wrapper


def MotorCallback(func):
    def wrapper(self, msg):
        return func(self, msg.left_percent, msg.right_percent)

    return wrapper
