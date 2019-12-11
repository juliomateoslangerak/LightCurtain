# Main code for light curtains
# A CircuitPython based program to make Neopixels react to a distance measurement form a ToF sensor
# The idea is to have light effects on a Neopixels strip as a sliding door is opening.
# The position of the door is detected by the distance sensor and feeds back to as a funcion than can 
# be used by some wave generators that will drive the efects on the neopixel strip

import pyb
# import busio
# from digitalio import DigitalInOut, Direction, Pull
# import adafruit_v15310x
# import neopixel

# import time
from waves import Signal, TransformedSignal, DecayWave, SquareWave

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

class FrameIntensity(Signal):
    def __init__(self, intensity):
        self._intensity = intensity

    def update(self, new_intensity):
        self._intensity = new_intensity

    def __call__(self):
        return self._intensity


class FramePhase(Signal):
    def __init__(self, phase=0.0):
        self._current_phase = phase

    def update(self, LED, nr_of_leds):
        self._current_phase = LED / nr_of_leds

    def __call__(self):
        return self._current_phase


class FrameFrequency(Signal):
    def __init__(self, frequency=1.0):
        self._current_frequency = frequency

    def update(self, frequency):
        self._current_frequency = frequency

    def __call__(self):
        return self._current_frequency


class FrameDecay(Signal):
    def __init__(self, decay=1.0):
        self._current_decay = decay

    def update(self, decay):
        self._current_decay = decay

    def __call__(self):
        return self._current_decay


class FrameDuty(Signal):
    def __init__(self, duty=0.5):
        self._current_duty = duty

    def update(self, duty):
        self._current_duty = duty

    def __call__(self):
        return self._current_duty


class FrameClock(Signal):
    def __init__(self):
        self._init_s = self._current_s = pyb.millis()
        self._stop_at = 0.0

    def init(self, stop_at):
        self._init_s = pyb.millis()
        self._current_s = pyb.elapsed_millis(self._init_s)
        self._stop_at = stop_at

    def update(self, speed=1.0):
        self._current_s = pyb.elapsed_millis(self._init_s) * speed

    def __call__(self):
        return min(self._current_s, self._stop_at)


class FrameSensor(Signal):
    """A class to manage the distance sensors reading"""

    def __init__(self, sensor_connection, calibration=1, distance=0):
        self._current_d = distance
        self._sensor_connection = sensor_connection
        self._calibration = calibration

    def update(self):
        self._current_d = self._sensor_connection.range * self._calibration

    def __call__(self):
        return self._current_d


class LED_curtains:
    def __init__(self,
                 neopixels_pin,
                 led_number,
                 curtain_start,
                 curtain_reverse,
                 curtain_sensors,
                 curtain_offsets,
                 mirror_relative_width,
                 mirror_offset,
                 start_threshold,
                 timed_wave_stop,
                 power,
                 ):
        """
        :type led_number: iter[int]
        :param led_number: list containing the number of neopixels in each curtain
        :param neopixels_pin: pin to which the neopixels are connected
        :param curtain_start: Neopixel start position for each curtain
        :param curtain_reverse: 1 or -1 tuple indicating if the curtains are to be reversed (1=Not, -1=Yes)
        :param curtain_sensors: List containing the sensor objects
        :param curtain_offsets: List containing a calibration factor for the ratio distance-phase
        :param mirror_relative_width: Relative width of the mirror lighting
        :param start_threshold: At what distance will the effect start or stop
        :param timed_wave_stop: time it takes for the white light to reach full intensity
        :param power: the max power we want to drive the neopixels. int from 0 to 255
        """
        self.power = power
        self.LED_number = led_number
        self.curtain_number = len(led_number)
        self.curtain_start = curtain_start
        self.curtain_reverse = curtain_reverse
        self.curtain_sensors = curtain_sensors
        self.curtain_offsets = curtain_offsets
        self.mirror_relative_width = mirror_relative_width
        self.mirror_offset = mirror_offset
        self.totalLEDs = sum(led_number)
        self.start_threshold = start_threshold
        self.timed_wave_stop = timed_wave_stop

        # Allocate some temporary color values
        self.red = 0
        self.green = 0
        self.blue = 0
        self.white = 0

        # create neopixels strip
        self.led_strip = neopixel.NeoPixel(pin=neopixels_pin,
                                           n=self.totalLEDs,
                                           bpp=4,
                                           auto_write=False)

        # Some frames
        self.clock = FrameClock()
        self.sensors = []
        for sensor in self.curtain_sensors:
            self.sensors.append(FrameSensor(sensor))
        self.frequency = FrameFrequency()
        self.decay = FrameDecay()
        self.curtain_phase = FramePhase()
        self.mirror_phase = FramePhase()
        self.duty = FrameDuty()
        self.red_curtain_intensity = FrameIntensity(self.power[0])
        self.green_curtain_intensity = FrameIntensity(self.power[1])
        self.blue_curtain_intensity = FrameIntensity(self.power[2])
        self.white_curtain_intensity = FrameIntensity(self.power[3])
        self.red_white_intensity = FrameIntensity(self.power[0])
        self.green_white_intensity = FrameIntensity(self.power[1])
        self.blue_white_intensity = FrameIntensity(self.power[2])
        self.white_white_intensity = FrameIntensity(self.power[3])
        self.red_mirror_intensity = FrameIntensity(self.power[0])
        self.green_mirror_intensity = FrameIntensity(self.power[1])
        self.blue_mirror_intensity = FrameIntensity(self.power[2])
        self.white_mirror_intensity = FrameIntensity(self.power[3])

        # Some basic waves
        self.red_curtain_wave = TransformedSignal(DecayWave(time=self.sensor,
                                                            frequency=self.frequency,
                                                            phase=self.curtain_phase,
                                                            decay=self.decay,
                                                            ),
                                                  y0=0,
                                                  y1=self.red_curtain_intensity,
                                                  discrete=True)

        self.green_curtain_wave = TransformedSignal(DecayWave(time=self.sensor,
                                                              frequency=self.frequency,
                                                              phase=self.curtain_phase,
                                                              decay=self.decay
                                                              ),
                                                    y0=0,
                                                    y1=self.green_curtain_intensity,
                                                    discrete=True)

        self.blue_curtain_wave = TransformedSignal(DecayWave(time=self.sensor,
                                                             frequency=self.frequency,
                                                             phase=self.curtain_phase,
                                                             decay=self.decay
                                                             ),
                                                   y0=0,
                                                   y1=self.blue_curtain_intensity,
                                                   discrete=True)

        self.white_curtain_wave = TransformedSignal(DecayWave(time=self.sensor,
                                                              frequency=self.frequency,
                                                              phase=self.curtain_phase,
                                                              decay=self.decay
                                                              ),
                                                    y0=0,
                                                    y1=self.white_curtain_intensity,
                                                    discrete=True)

        self.red_timed_wave = TransformedSignal(DecayWave(time=self.clock,
                                                          frequency=self.frequency,
                                                          phase=self.curtain_phase,
                                                          decay=self.decay,
                                                          ),
                                                y0=0,
                                                y1=self.red_white_intensity,
                                                discrete=True)

        self.green_timed_wave = TransformedSignal(DecayWave(time=self.clock,
                                                            frequency=self.frequency,
                                                            phase=self.curtain_phase,
                                                            decay=self.decay
                                                            ),
                                                  y0=0,
                                                  y1=self.green_white_intensity,
                                                  discrete=True)

        self.blue_timed_wave = TransformedSignal(DecayWave(time=self.clock,
                                                           frequency=self.frequency,
                                                           phase=self.curtain_phase,
                                                           decay=self.decay
                                                           ),
                                                 y0=0,
                                                 y1=self.blue_white_intensity,
                                                 discrete=True)

        self.white_timed_wave = TransformedSignal(DecayWave(time=self.clock,
                                                            frequency=self.frequency,
                                                            phase=self.curtain_phase,
                                                            decay=self.decay
                                                            ),
                                                  y0=0,
                                                  y1=self.white_white_intensity,
                                                  discrete=True)

        self.red_mirror_wave = TransformedSignal(SquareWave(time=self.sensor,
                                                            frequency=self.frequency,
                                                            phase=self.mirror_phase,
                                                            duty=self.duty,
                                                            ),
                                                 y0=0,
                                                 y1=self.red_mirror_intensity,
                                                 discrete=True)

        self.green_mirror_wave = TransformedSignal(SquareWave(time=self.sensor,
                                                              frequency=self.frequency,
                                                              phase=self.mirror_phase,
                                                              duty=self.duty,
                                                              ),
                                                   y0=0,
                                                   y1=self.green_mirror_intensity,
                                                   discrete=True)

        self.blue_mirror_wave = TransformedSignal(SquareWave(time=self.sensor,
                                                             frequency=self.frequency,
                                                             phase=self.mirror_phase,
                                                             duty=self.duty,
                                                             ),
                                                  y0=0,
                                                  y1=self.blue_mirror_intensity,
                                                  discrete=True)

        self.white_mirror_wave = TransformedSignal(SquareWave(time=self.sensor,
                                                              frequency=self.frequency,
                                                              phase=self.mirror_phase,
                                                              duty=self.duty
                                                              ),
                                                   y0=0,
                                                   y1=self.white_mirror_intensity,
                                                   discrete=True)

    def set_white(self):
        self.led_strip.fill(self.power)
        self.led_strip.show()

    def set_off(self):
        self.led_strip.fill((0, 0, 0, 0))
        self.led_strip.show()

    def single_pulse(self, pulse_color, t=0.2):
        self.led_strip.fill(pulse_color)
        self.led_strip.show()
        time.sleep(float(t))
        self.led_strip.fill((0, 0, 0, 0))
        self.led_strip.show()

    def run_curtain(self,
                    curtain_color,
                    white_color=(0, 0, 0, 127),
                    mirror_color=(0, 0, 0, 255),
                    decay=1.0,
                    frequency=1.0
                    ):
        """
        Creates a 'color' LED chasing effect followed by brigt white-level. 
        The full white level arrives anyway after a certain time defined by frequency.
        :param curtain_color: tupple with the color to display
        :param white_color: white color that will be reached after wave
        :param mirror_color: white color that will be reached on mirror
        :param decay: the decay factor
        :param frequency: how many turns per second. Defaults to 1
        :return: None
        """
        self.red_curtain_intensity.update(curtain_color[0])
        self.green_curtain_intensity.update(curtain_color[1])
        self.blue_curtain_intensity.update(curtain_color[2])
        self.white_curtain_intensity.update(curtain_color[3])
        self.red_white_intensity.update(white_color[0])
        self.green_white_intensity.update(white_color[1])
        self.blue_white_intensity.update(white_color[2])
        self.white_white_intensity.update(white_color[3])
        self.red_mirror_intensity.update(mirror_color[0])
        self.green_mirror_intensity.update(mirror_color[1])
        self.blue_mirror_intensity.update(mirror_color[2])
        self.white_mirror_intensity.update(mirror_color[3])

        self.clock.init(stop_at=self.timed_wave_stop)
        self.frequency.update(frequency)
        self.decay.update(decay)

        while True:
            # First the time for all operations is set
            self.clock.update()
            # And the sensors readings are updated
            for s in self.curtain_sensors:
                s.update()
            # Exit if the doors are closed
            if any([s() < self.start_threshold for s in self.curtain_sensors]):
                self.led_strip.fill([0, 0, 0, 0])
                self.led_strip.show()
                return
            # We set intensities for every curtain
            for curtain in range(self.curtain_number):
                for led in range(self.LED_number[curtain]):
                    self.curtain_phase.update(led + self.curtain_offsets[curtain], self.LED_number[curtain])
                    self.mirror_phase.update(led + self.mirror_offset, self.LED_number[curtain])
                    self.red = max(self.red_curtain_wave(), 255 - self.red_timed_wave(), self.red_mirror_wave())
                    self.green = max(self.green_curtain_wave(), 255 - self.green_timed_wave(), self.green_mirror_wave())
                    self.blue = max(self.blue_curtain_wave(), 255 - self.blue_timed_wave(), self.blue_mirror_wave())
                    self.white = max(self.white_curtain_wave(), 255 - self.white_timed_wave(), self.white_mirror_wave())
                    self.led_strip[self.curtain_start[curtain] +
                                   led * self.curtain_reverse[curtain]] = (self.red,
                                                                           self.green,
                                                                           self.blue,
                                                                           self.white)
            self.led_strip.show()
            # time.sleep(0.1)


if __name__ == '__main__':
    i2c = busio.I2C(board.SCL, board.SDA)

    xshut_left = pyb.Pin('D0', pyb.Pin.OUT)
    xshut_right = pyb.Pin('D1', pyb.Pin.OUT)
    pixels = pyb.Pin('D2', pyb.Pin.OUT)

    # Reset all the sensors
    xshut_left.low()
    xshut_right.low()
    pyb.delay(10)
    xshut_left.high()
    xshut_right.high()
    pyb.delay(10)

    # Initialize the sensors one at a time so we can assign them different addresses
    xshut_right.low()
    sensor_left = adafruit_vl5310x.VL53L0X(i2c, I2C_ADDRESS_LEFT, TIMEOUT)
    xshut_right.high()
    sensor_right = adafruit_v15310x.VL53L0X(i2c, I2C_ADDRESS_RIGHT, TIMEOUT)

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
