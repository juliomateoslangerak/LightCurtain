import board
import busio
from digitalio import DigitalInOut, Direction, Pull
import adafruit_vl53l0x
import time
from Curtains import LED_curtains

# main parameters
CURTAIN_COLOR = (255, 0, 0, 0)

# config parameters
TIMEOUT = 0
I2C_ADDRESS_LEFT = 48  # address 0x30
I2C_ADDRESS_RIGHT = 49  # address 0x31
READ_TIME = 50000  # the read time of the sensors in usec. Higher numbers provide higher accuracy but will be slower

# curtain size configuration
LED_NUMBER = (140, 140)
CURTAIN_START = (0, 280)
CURTAIN_REVERSE = (1, -1)
CURTAIN_OFFSETS = (0, 0)
MIRROR_RELATIVE_WIDTH = 0.6
MIRROR_OFFSET = (0, 0)
START_THRESHOLD = 20
TIMED_WAVE_STOP = 5000
POWER = (128, 128, 128, 128)

i2c = busio.I2C(board.SCL, board.SDA)

xshut_left = DigitalInOut(board.D1)
xshut_left.direction = Direction.OUTPUT
xshut_right = DigitalInOut(board.D3)
xshut_right.direction = Direction.OUTPUT
pixels = DigitalInOut(board.D4)
pixels.direction = Direction.OUTPUT

# Reset all the sensors
xshut_left.value = False
xshut_right.value = False
time.sleep(.01)
xshut_left.value = True
xshut_right.value = True
time.sleep(.01)

# Initialize the sensors one at a time so we can assign them different addresses
xshut_right.value = False
sensor_left = adafruit_vl53l0x.VL53L0X(i2c, I2C_ADDRESS_LEFT, TIMEOUT)
xshut_right.value = True
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, I2C_ADDRESS_RIGHT, TIMEOUT)

# change read time
sensor_left.measurement_timing_budget = READ_TIME
sensor_right.measurement_timing_budget = READ_TIME

Curtains = LED_curtains(neopixels_pin=pixels,
                        led_number=LED_NUMBER,
                        curtain_start=CURTAIN_START,
                        curtain_reverse=CURTAIN_REVERSE,
                        curtain_sensors=[sensor_right, sensor_left],
                        curtain_offsets=CURTAIN_OFFSETS,
                        mirror_relative_width=MIRROR_RELATIVE_WIDTH,
                        mirror_offset=MIRROR_OFFSET,
                        start_threshold=START_THRESHOLD,
                        timed_wave_stop=TIMED_WAVE_STOP,
                        power=POWER)

Curtains.test_strip([(255,0,0,0), (0, 255, 0, 0), (0, 0, 255, 0), (0, 0, 0, 255)])

while True:
    if any(sensor_left.range > START_THRESHOLD,
           sensor_right.range > START_THRESHOLD):
        Curtains.run_curtain(curtain_color=CURTAIN_COLOR)
    time.sleep(.1)
