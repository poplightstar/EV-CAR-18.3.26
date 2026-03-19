from microbit import i2c, sleep
from microbit import display, Image

CHIP_ADDRESS = 0x6C

def initPCA():
    i2c.write(CHIP_ADDRESS, bytearray([0xFE, 0x7D]))
    for reg in range(0xFA, 0xFE):
        i2c.write(CHIP_ADDRESS, bytearray([reg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([0x00, 0x01]))
    sleep(10)

SERVO_REG_BASE = 0x06
BYTES_PER_CHANNEL = 4

def set_servo(servo, degrees):
    min_pulse = 178
    max_pulse = 416
    if degrees < 0:
        degrees = 0
    if degrees > 120:
        degrees = 120
    pulse = min_pulse + int((max_pulse - min_pulse) * degrees / 120)
    reg = SERVO_REG_BASE + (servo - 1) * BYTES_PER_CHANNEL
    i2c.write(CHIP_ADDRESS, bytearray([reg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 1, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 2, pulse & 0xFF]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 3, pulse >> 8]))

CLAW_SERVO = 1
initPCA()

while True:
    set_servo(CLAW_SERVO, 0)
    display.show("0")
    sleep(4000)
    set_servo(CLAW_SERVO, 120)
    display.show("1")
    sleep(4000)
