from microbit import *
import radio
import utime
# ----------------- Images -----------------
F = Image.ARROW_N
R = Image.ARROW_S
Lt = Image.ARROW_W
Rt = Image.ARROW_E
# ----------------- PCA9685 / Motor setup -----------------
CHIP_ADDRESS = 108
MOT_REG_BASE = 0x28
REG_OFFSET = 4
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
    return MOT_REG_BASE + (2 * (motor - 1) * REG_OFFSET)
def motor_forwards(motor, speed):
    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
    motorReg = _motor_base(motor)
    pwm = int(speed * 40.95)
    low = pwm & 0xFF
    high = pwm >> 8
    revReg = motorReg + 4
    i2c.write(CHIP_ADDRESS, bytearray([revReg, low]))
    i2c.write(CHIP_ADDRESS, bytearray([revReg + 1, high]))
    i2c.write(CHIP_ADDRESS, bytearray([motorReg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([motorReg + 1, 0x00]))
def motor_reverse(motor, speed):
    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
    motorReg = _motor_base(motor)
    pwm = int(speed * 40.95)
    low = pwm & 0xFF
    high = pwm >> 8
    revReg = motorReg + 4
    i2c.write(CHIP_ADDRESS, bytearray([motorReg, low]))
    i2c.write(CHIP_ADDRESS, bytearray([motorReg + 1, high]))
    i2c.write(CHIP_ADDRESS, bytearray([revReg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([revReg + 1, 0x00]))
def stop_all_motors():
    for m in (1, 2, 3, 4):
        motor_forwards(m, 0)
        motor_reverse(m, 0)
def forwards(speed):
    motor_forwards(1, speed)
    motor_forwards(2, speed)
    motor_forwards(3, speed)
    motor_forwards(4, speed)
def reverse(speed):
    motor_reverse(1, speed)
    motor_reverse(2, speed)
    motor_reverse(3, speed)
    motor_reverse(4, speed)
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
# ----------------- Ultrasonic + LEDs -----------------
TRIG = pin0
ECHO = pin1
RED = pin15
YELLOW = pin14
GREEN = pin13
SAFE_DIST = 30
CAUTION_DIST = 20
SAFE_SPEED = 80
CAUTION_SPEED = 40
LINE_SPEED = 60
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
    end = utime.ticks_us()
    duration = end - start
    distance = (duration / 2.0) / 29.1
    return distance
# ----------------- Line following -----------------
LINE_LEFT = pin16
LINE_RIGHT = pin12
LINE_LEFT.set_pull(LINE_LEFT.PULL_UP)
LINE_RIGHT.set_pull(LINE_RIGHT.PULL_UP)
def read_line_sensors():
    return LINE_LEFT.read_digital(), LINE_RIGHT.read_digital()
def line_follow_step(speed):
    left, right = read_line_sensors()
    if left == 1 and right == 1:
        forwards(speed)
        display.show(F)
        return True
    elif left == 1 and right == 0:
        arc_left(speed)
        display.show(Lt)
        return True
    elif left == 0 and right == 1:
        arc_right(speed)
        display.show(Rt)
        return True
    else:
        stop_all_motors()
        display.show(Image.SQUARE_SMALL)
        return False
# ----------------- Init + Main loop -----------------
initPCA()
stop_all_motors()
set_leds(0, 0, 0)
radio.config(group=5)
radio.on()
current_speed = SAFE_SPEED
line_follow_mode = False
while True:
    d = measure_distance()
    obstacle_too_close = False
    if d == -1:
        set_leds(0, 1, 0)
        current_speed = SAFE_SPEED
    else:
        d = int(d)
        if d >= SAFE_DIST:
            set_leds(0, 0, 1)
            current_speed = SAFE_SPEED
        elif CAUTION_DIST < d < SAFE_DIST:
            set_leds(0, 1, 0)
            current_speed = CAUTION_SPEED
        else:
            set_leds(1, 0, 0)
            stop_all_motors()
            obstacle_too_close = True
            current_speed = 0
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
                display.show(F)
            elif message == "reverse":
                reverse(current_speed)
                display.show(R)
            elif message == "turn_left":
                turn_left(current_speed)
                display.show(Lt)
            elif message == "turn_right":
                turn_right(current_speed)
                display.show(Rt)
            elif message == "stop":
                stop_all_motors()
                display.clear()
    sleep(50)
