from microbit import *
import radio
import utime

CHIP_ADDRESS = 108
MOT_REG_BASE = 0x28
BYTES_PER_CHANNEL = 4
CHANNELS_PER_MOTOR = 2


def initPCA():
    i2c.write(CHIP_ADDRESS, bytearray([0x00, 0x00]))
    sleep(10)
    i2c.write(CHIP_ADDRESS, bytearray([0xFE, 0x79]))
    sleep(10)
    i2c.write(CHIP_ADDRESS, bytearray([0xFA, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([0xFB, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([0xFC, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([0xFD, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([0x00, 0x01]))
    sleep(10)


def _motor_base(motor):
    return MOT_REG_BASE + (motor - 1) * CHANNELS_PER_MOTOR * BYTES_PER_CHANNEL


def motor_forwards(motor, speed):
    speed = max(0, min(100, speed))
    motorReg = _motor_base(motor)
    pwm = int(speed * 40.95)
    low = pwm & 0xFF
    high = pwm >> 8
    revReg = motorReg + 4
    i2c.write(CHIP_ADDRESS, bytearray([motorReg, low]))
    i2c.write(CHIP_ADDRESS, bytearray([motorReg + 1, high]))
    i2c.write(CHIP_ADDRESS, bytearray([revReg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([revReg + 1, 0x00]))


def motor_reverse(motor, speed):
    speed = max(0, min(100, speed))
    motorReg = _motor_base(motor)
    pwm = int(speed * 40.95)
    low = pwm & 0xFF
    high = pwm >> 8
    revReg = motorReg + 4
    i2c.write(CHIP_ADDRESS, bytearray([revReg, low]))
    i2c.write(CHIP_ADDRESS, bytearray([revReg + 1, high]))
    i2c.write(CHIP_ADDRESS, bytearray([motorReg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([motorReg + 1, 0x00]))


def stop_all_motors():
    for m in (1, 2, 3, 4):
        motor_forwards(m, 0)


def forwards(speed):
    for m in (1, 2, 3, 4):
        motor_forwards(m, speed)


def reverse(speed):
    for m in (1, 2, 3, 4):
        motor_reverse(m, speed)


def turn_right(speed):
    motor_reverse(1, speed)
    motor_reverse(2, speed)
    motor_forwards(3, speed)
    motor_forwards(4, speed)


def turn_left(speed):
    motor_reverse(3, speed)
    motor_reverse(4, speed)
    motor_forwards(1, speed)
    motor_forwards(2, speed)


# --- Gentle arc turns for line following ---
def arc_left(speed):
    motor_forwards(1, speed // 4)
    motor_forwards(2, speed // 4)
    motor_forwards(3, speed)
    motor_forwards(4, speed)


def arc_right(speed):
    motor_forwards(1, speed)
    motor_forwards(2, speed)
    motor_forwards(3, speed // 4)
    motor_forwards(4, speed // 4)


# --- Klaw (servo claw on pin16) ---
# NOTE: pin16 is shared with the line-following left sensor.
# You cannot use line following and the claw at the same time.
class Klaw:
    def __init__(self, pin, freq=50, min_us=700, max_us=2300, angle=180):
        self.pin = pin
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us
        self.angle = angle
        self.us = 0
        analog_period = round((1 / self.freq) * 1000)
        self.pin.set_analog_period(analog_period)

    def write_us(self, us):
        self.us = min(self.max_us, max(self.min_us, us))
        analog_op = round(self.us * 1024 * self.freq // 1000000)
        self.pin.write_analog(analog_op)

    def write_angle(self, degrees):
        degrees = degrees // 1
        pulse_range = self.max_us - self.min_us
        self.us = self.min_us + (pulse_range * degrees) // self.angle
        self.write_us(self.us)

    def stop(self):
        self.pin.write_digital(0)


# --- Pins ---
TRIG = pin0
ECHO = pin1
RED = pin8
YELLOW = pin14
GREEN = pin13

SAFE_DIST = 30
CAUTION_DIST = 20
SAFE_SPEED = 80
CAUTION_SPEED = 40
LINE_SPEED = 60

CLAW_OPEN_ANGLE = 180
CLAW_CLOSE_ANGLE = 0


def set_leds(r, y, g):
    RED.write_digital(1 if r else 0)
    YELLOW.write_digital(1 if y else 0)
    GREEN.write_digital(1 if g else 0)


def measure_distance():
    TRIG.write_digital(0)
    utime.sleep_us(2)
    TRIG.write_digital(1)
    utime.sleep_us(10)
    TRIG.write_digital(0)
    timeout = utime.ticks_us() + 30000
    while ECHO.read_digital() == 0:
        if utime.ticks_us() > timeout:
            return -1
    start = utime.ticks_us()
    timeout = utime.ticks_us() + 30000
    while ECHO.read_digital() == 1:
        if utime.ticks_us() > timeout:
            return -1
    return (utime.ticks_us() - start) / 2.0 / 29.1


# NOTE: pin16 is shared between LINE_LEFT and the Klaw servo.
# Line sensor pull-ups are set up in main, AFTER the Klaw is initialised,
# so that the Klaw can configure pin16 for analog output first.
LINE_LEFT = pin16
LINE_RIGHT = pin15


def read_line_sensors():
    return LINE_LEFT.read_digital(), LINE_RIGHT.read_digital()


def line_follow_step(speed):
    left, right = read_line_sensors()
    if left == 1 and right == 1:
        forwards(speed)
        display.show(Image.ARROW_N)
        return True
    elif left == 1 and right == 0:
        arc_left(speed)
        display.show(Image.ARROW_W)
        return True
    elif left == 0 and right == 1:
        arc_right(speed)
        display.show(Image.ARROW_E)
        return True
    else:
        stop_all_motors()
        display.show(Image.SQUARE_SMALL)
        return False


# --- Main ---
initPCA()
stop_all_motors()
set_leds(0, 0, 0)
radio.config(group=5)
radio.on()

my_klaw = Klaw(pin16)
my_klaw.write_angle(CLAW_OPEN_ANGLE)

# Set up line sensor pull-ups AFTER Klaw init (pin16 is shared)
LINE_RIGHT.set_pull(LINE_RIGHT.PULL_UP)

current_speed = SAFE_SPEED
obstacle_too_close = False
line_follow_mode = False

while True:
    d = measure_distance()
    if d == -1:
        set_leds(0, 0, 1)
        current_speed = SAFE_SPEED
        obstacle_too_close = False
    else:
        d = int(d)
        if d >= SAFE_DIST:
            set_leds(0, 0, 1)
            current_speed = SAFE_SPEED
            obstacle_too_close = False
        elif d > CAUTION_DIST:
            set_leds(0, 1, 0)
            current_speed = CAUTION_SPEED
            obstacle_too_close = False
        else:
            set_leds(1, 0, 0)
            stop_all_motors()
            current_speed = 0
            obstacle_too_close = True

    message = radio.receive()
    if message == "line_mode":
        line_follow_mode = not line_follow_mode
        stop_all_motors()
        if line_follow_mode:
            display.show(Image.TARGET)
        else:
            display.show(Image.NO)
        sleep(400)


    if not obstacle_too_close:
        if line_follow_mode:
            line_follow_step(LINE_SPEED)
        elif message is not None and message != "line_mode":
            if message == "forwards":
                forwards(current_speed)
                display.show(Image.ARROW_N)
            elif message == "reverse":
                reverse(current_speed)
                display.show(Image.ARROW_S)
            elif message == "turn_left":
                turn_left(current_speed)
                display.show(Image.ARROW_W)
            elif message == "turn_right":
                turn_right(current_speed)
                display.show(Image.ARROW_E)
            elif message == "stop":
                stop_all_motors()
                display.clear()

    sleep(50)
