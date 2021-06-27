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
            value = 1 if int(value_txt) else 0

            if pin not in pins:
                print('Unknown PIN', pin, 'expect one of', pins, file=sys.stderr)
                continue

            redboard.output_pin(pin)
            redboard.setPin(pin, value)
    finally:
        for pin in pins:
            redboard.pull_up(pin)


if __name__ == '__main__':
    script()
