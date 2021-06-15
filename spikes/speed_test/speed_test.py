import sys
import redboard


def script():
    try:
        while True:
            speed_txt = input('speed> ')
            if speed_txt.strip() == '':
                break

            try:
                speed = float(speed_txt)
            except ValueError:
                print('Enter a number', flush=True, file=sys.stderr)
                continue

            redboard.M1(speed)
            redboard.M2(-speed)

    finally:
        # Ensure we stop on exit
        redboard.M1(0)
        redboard.M2(0)


if __name__ == '__main__':
    script()
