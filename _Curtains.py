import neopixel
import time
from waves import Signal, TransformedSignal, DecayWave, SineWave, SquareWave
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
  self._init_s = self._current_s = time.time()
  self._stop_at = 0.0
 def init(self, stop_at):
  self._init_s = time.time()
  self._stop_at = stop_at
 def update(self, speed=1.0):
  self._current_s = (time.time() - self._init_s) * speed
 def __call__(self):
  return min(self._current_s, self._stop_at)
class FrameSensor(Signal):
 def __init__(self, sensor_connection, calibration=1, distance=0):
  self._current_d = distance
  self._sensor_connection = sensor_connection
  self._calibration = calibration
 def update(self):
  self._current_d = self._sensor_connection.range * self._calibration
 def __call__(self):
  return self._current_d
class LED_curtains:
 def __init__(self,neopixels_pin,led_number,curtain_start,curtain_reverse,curtain_sensors,curtain_offsets,mirror_relative_width,mirror_offset,start_threshold,timed_wave_stop,power,):
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
  self.red = 0
  self.green = 0
  self.blue = 0
  self.white = 0
  self.led_strip = neopixel.NeoPixel(pin=neopixels_pin,n=self.totalLEDs,bpp=4,auto_write=False)
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
  self.red_curtain_wave = TransformedSignal(DecayWave(time=self.sensor,frequency=self.frequency,phase=self.curtain_phase,decay=self.decay,),
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
 ),y0=0,y1=self.blue_mirror_intensity,discrete=True)
  self.white_mirror_wave = TransformedSignal(SquareWave(time=self.sensor,frequency=self.frequency,phase=self.mirror_phase,duty=self.duty),y0=0,y1=self.white_mirror_intensity,discrete=True)
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
 def test_strip(self, test_pattern):
  for p in test_pattern:
   for i in range(self.totalLEDs):
    self.led_strip[i] = p
    self.led_strip.show()
    time.sleep(.1)
    self.led_strip[i] = (0, 0, 0, 0)
    self.led_strip.show()
 def run_curtain(self,curtain_color,white_color=(0, 0, 0, 127),mirror_color=(0, 0, 0, 255),decay=1.0,frequency=1.0):
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
   self.clock.update()
   for s in self.curtain_sensors:
    s.update()
   if any([s() < self.start_threshold for s in self.curtain_sensors]):
    self.led_strip.fill([0, 0, 0, 0])
    self.led_strip.show()
    return
   for curtain in range(self.curtain_number):
    for led in range(self.LED_number[curtain]):
     self.curtain_phase.update(led + self.curtain_offsets[curtain], self.LED_number[curtain])
     self.mirror_phase.update(led + self.mirror_offset, self.LED_number[curtain])
     self.red = max(self.red_curtain_wave(), 255 - self.red_timed_wave(), self.red_mirror_wave())
     self.green = max(self.green_curtain_wave(), 255 - self.green_timed_wave(), self.green_mirror_wave())
     self.blue = max(self.blue_curtain_wave(), 255 - self.blue_timed_wave(), self.blue_mirror_wave())
     self.white = max(self.white_curtain_wave(), 255 - self.white_timed_wave(), self.white_mirror_wave())
     self.led_strip[self.curtain_start[curtain] + led * self.curtain_reverse[curtain]] = (self.red,
     self.green,
     self.blue,
     self.white)
   self.led_strip.show()
