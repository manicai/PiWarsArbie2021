from bottle import route, template
import bottle
import time

try:
    import smbus
    redboard_bus = smbus.SMBus(1)
    redboard_active = True
except ImportError:
    smbus = None
    redboard_bus = None
    redboard_active = False


def get_temperature():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as fh:
            millicelsius = fh.read()
            celsius = int(millicelsius) / 1000.0
            return celsius
    except Exception as exc:
        print(exc)
        return None


def get_battery_voltage():
    # Based on Redboard bat_check.py
    if redboard_bus is None:
        return None

    adc_bat_conversion_value = 1098.0

    address = 0x48
    redboard_bus.write_i2c_block_data(address, 0x01, [0xc3, 0x83])
    time.sleep(0.01)
    voltage0 = redboard_bus.read_i2c_block_data(address, 0x00, 2)

    # Battery Voltage
    conversion_0 = (voltage0[1]) + (voltage0[0] << 8)
    volts_0 = conversion_0 / adc_bat_conversion_value

    return volts_0


def get_ip_address():
    import socket
    return socket.gethostbyname(socket.gethostname())


@route('/')
def index():
    return template('status_index.html',
                    redboard_active=redboard_active,
                    cpu_temperature=get_temperature(),
                    battery_voltage=get_battery_voltage(),
                    ip_address=get_ip_address())


if __name__ == '__main__':
    bottle.debug(True)
    bottle.TEMPLATE_PATH = ['templates']
    bottle.run(host='0.0.0.0', port=5000, reloader=True)
