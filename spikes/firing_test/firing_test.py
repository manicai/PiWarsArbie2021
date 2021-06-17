import sys
import redboard

pins = [20, 21]


def script():
    try:
        while True:
            input_txt = input('pin value> ')
            if input_txt.strip() == '':
                break

            pin_txt, value_txt = input_txt.split(' ', 2)
            pin = int(pin_txt)
            value = int(value_txt)

            if pin not in pins:
                print('Unknown PIN', pin, 'expect one of', pins, file=sys.stderr)
                continue

            if not value:
                # Set solenoid
                redboard.pull_down(pin)
            else:
                # Unset solenoid
                redboard.pull_up(pin)

    finally:
        for pin in pins:
            redboard.pull_up(pin)


if __name__ == '__main__':
    script()
