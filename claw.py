from microbit import i2c, sleep
from microbit import display, Image

# --- PCA9685 Setup (matched to Marcel's) ---
CHIP_ADDRESS = 0x6C

def initPCA():
    i2c.write(CHIP_ADDRESS, bytearray([0xFE, 0x7D]))
    for reg in range(0xFA, 0xFE):
        i2c.write(CHIP_ADDRESS, bytearray([reg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([0x00, 0x01]))
    sleep(10)

# --- Servo Control ---
# Servo ports use PCA9685 channels 0-7
# Registers start at 0x06, 4 bytes per channel
SERVO_REG_BASE = 0x06
BYTES_PER_CHANNEL = 4

def set_servo(servo, degrees):
    # With 0x7D prescaler, frequency is ~50Hz (20ms period)
    # 4096 counts per 20ms period
    # 1ms pulse (~0 deg) = 204 counts
    # 1.5ms pulse (~90 deg) = 307 counts
    # 2ms pulse (~180 deg) = 410 counts
    min_pulse = 150
    max_pulse = 450
    pulse = min_pulse + int((max_pulse - min_pulse) * degrees / 180)
    reg = SERVO_REG_BASE + (servo - 1) * BYTES_PER_CHANNEL
    i2c.write(CHIP_ADDRESS, bytearray([reg, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 1, 0x00]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 2, pulse & 0xFF]))
    i2c.write(CHIP_ADDRESS, bytearray([reg + 3, pulse >> 8]))

# --- Main ---
CLAW_SERVO = 1

initPCA()
set_servo(CLAW_SERVO, 180)
display.show(Image.YES)
