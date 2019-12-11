import math
class Signal:
    @property
    def range(self):
        return None
    def __call__(self):
        raise NotImplementedError('Signal must have a callable implementation!')
    def transform(self, y0, y1):
        x = self()
        if callable(y0):
            y0 = y0()
        if callable(y1):
            y1 = y1()
        if self.range is not None:
            return y0 + (x-self.range[0]) * \
                        ((y1-y0)/(self.range[1]-self.range[0]))
        else:
            return max(y0, min(y1, x))
    def discrete_transform(self, y0, y1):
        return int(self.transform(y0, y1))
class SignalSource:
    def __init__(self, source=None):
        self.set_source(source)
    def __call__(self):
        return self._source()
    def set_source(self, source):
        if callable(source):
            self._source = source
        else:
            self._source = lambda: source
class SineWave(Signal):
    def __init__(self, time=0.0, amplitude=1.0, frequency=1.0, phase=0.0):
        self.time = SignalSource(time)
        self.amplitude = SignalSource(amplitude)
        self.frequency = SignalSource(frequency)
        self.phase = SignalSource(phase)
    @property
    def range(self):
        amplitude = self.amplitude()
        return -amplitude, amplitude
    def __call__(self):
        return self.amplitude() * \
               math.sin(2*math.pi*self.frequency()*self.time() + self.phase())
class SquareWave(Signal):
    def __init__(self, time=0.0, amplitude=1.0, frequency=1.0, phase=0.0, duty=0.5):
        self.time = SignalSource(time)
        self.amplitude = SignalSource(amplitude)
        self.frequency = SignalSource(frequency)
        self.phase = SignalSource(phase)
        self.duty = SignalSource(duty)
    @property
    def range(self):
        amplitude = self.amplitude()
        return -amplitude, amplitude
    def __call__(self):
        cycle = 1 / self.frequency()
        reminder = (self.time() + self.phase()) % cycle
        if reminder < self.duty() * cycle:
            return self.amplitude()
        else:
            return -1 * self.amplitude()
class DecayWave(Signal):
    def __init__(self, time=0.0, amplitude=1.0, frequency=1.0, phase=0.0, decay=0.0):
        self.time = SignalSource(time)
        self.amplitude = SignalSource(amplitude)
        self.frequency = SignalSource(frequency)
        self.phase = SignalSource(phase)
        self.decay = SignalSource(decay)
    @property
    def range(self):
        amplitude = self.amplitude()
        return 0, amplitude
    def __call__(self):
        reminder = (self.time() + self.phase()) % (1 / self.frequency())
        return self.amplitude() * math.exp(-1 * self.decay() * reminder)
class TransformedSignal(Signal):
    def __init__(self, source_signal, y0, y1, discrete=False):
        self.source = source_signal
        self.y0 = y0
        self.y1 = y1
        if not discrete:
            self._transform = self.source.transform
        else:
            self._transform = self.source.discrete_transform
    @property
    def range(self):
        return self.y0, self.y1
    def __call__(self):
        return self._transform(self.y0, self.y1)
