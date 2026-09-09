"""USB serial adapter and deterministic sensor replay using the same wire format."""
from .protocol import drive_command

class SerialTransport:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=.08, write_timeout=.15)
        self.buffer = bytearray()

    def read(self):
        chunk = self.serial.read_until(b'\n', 256)
        self.buffer.extend(chunk)
        if len(self.buffer) > 256:
            self.buffer.clear()
            return b''
        if self.buffer.endswith(b'\n'):
            result = bytes(self.buffer)
            self.buffer.clear()
            return result
        return b''

    def write(self, seq, left, right):
        self.serial.write(drive_command(seq, left, right))

    def close(self):
        try:
            self.write(0, 0, 0)
        finally:
            self.serial.close()

class MockTransport:
    """Replay: clear -> fog -> approaching obstacle -> hold -> clear -> park.

    Synthetic inputs exercise the real host pipeline; they are not sensor evidence.
    """
    def __init__(self):
        self.step = 0
        self.left_ticks = self.right_ticks = 0.0
        self.pwm = (0, 0)

    def read(self):
        import math
        self.step += 1
        t = self.step / 10
        scale = .1 * 20 / (math.pi * .065) / 450
        self.left_ticks += self.pwm[0] * scale
        self.right_ticks += self.pwm[1] * scale
        distance = 1800 if t < 5 else max(160, round(1800-(t-5)*550)) if t < 10 else 1800
        fog = int(3 <= t < 5)
        line = '850,850,850' if t >= 13 else '100,850,100'
        return f'S,{self.step},{self.step*100},{distance},{int(self.left_ticks)},{int(self.right_ticks)},{line},0,{fog},1,0,0,1000,0\n'.encode()

    def write(self, seq, left, right):
        drive_command(seq, left, right)
        self.pwm = (left, right)

    def close(self):
        self.pwm = (0, 0)
