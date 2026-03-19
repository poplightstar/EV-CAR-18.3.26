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
    # FS90 pulse range: 900us to 2100us
    # PCA9685 at 48.4Hz: 4096 counts per 20.66ms
    # 900us = 900/20660 * 4096 = 178 counts
    # 2100us = 2100/20660 * 4096 = 416 counts
    min_pulse = 178
    max_pulse = 416
    # FS90 actual range is 120 degrees
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
set_servo(CLAW_SERVO, 120)  # Full open
display.show(Image.YES)
