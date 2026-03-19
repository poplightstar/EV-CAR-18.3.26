from microbit import *
import utime

# ----------------- PCA9685 Setup -----------------
CHIP_ADDRESS = 108

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

# ----------------- Servo Control -----------------
# Servo ports on the Kitronik Robotics Board use PCA9685 channels 0-7
# Each channel has 4 registers starting at 0x06
SERVO_REG_BASE = 0x06
BYTES_PER_CHANNEL = 4

def set_servo(servo, degrees):
    # Servo pulse: 1ms (0 deg) to 2ms (180 deg) at 50Hz (20ms period)
    # PCA9685 full range = 4096 counts per 20ms
    # 1ms = ~205 counts, 2ms = ~410 counts
    min_pulse = 205
    max_pulse = 410
    pulse = min_pulse + int((max_pulse - min_pulse) * degrees / 180)
    reg = SERVO_REG_BASE + (servo - 1) * BYTES_PER_CHANNEL
    i2c.write(CHIP_ADDRESS, bytearray([reg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 1, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 2, pulse & 0xFF]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 3, pulse >> 8]))

# ----------------- Main -----------------
CLAW_SERVO = 1  # Servo port 1 on the Kitronik board

initPCA()

# Open the claw
set_servo(CLAW_SERVO, 180)
display.show(Image.YES)
