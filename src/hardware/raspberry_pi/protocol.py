"""Bounded serial parser shared by the real and mock transports."""
from dataclasses import dataclass

@dataclass(frozen=True)
class Sample:
    seq: int
    millis: int
    distance_m: float | None
    left_ticks: int
    right_ticks: int
    line: tuple[int, int, int]
    estop: bool
    fog: bool
    imu_ok: bool
    accel_g: tuple[float, float, float]
    yaw_rate_dps: float

def parse_sample(line: bytes) -> Sample:
    if len(line) > 256 or not line.endswith(b'\n'):
        raise ValueError('Incomplete or oversized telemetry')
    parts = line.decode('ascii').strip().split(',')
    if len(parts) != 16 or parts[0] != 'S':
        raise ValueError('Expected S plus 15 integer fields')
    v = [int(x) for x in parts[1:]]
    seq, millis, distance, left, right = v[:5]
    if not all(0 <= x <= 0xffffffff for x in (seq, millis, left, right)):
        raise ValueError('Invalid counter')
    if distance != -1 and not 20 <= distance <= 4000:
        raise ValueError('Invalid range')
    if not all(0 <= x <= 1023 for x in v[5:8]):
        raise ValueError('Invalid line sensor reading')
    if not all(x in (0, 1) for x in v[8:11]):
        raise ValueError('Invalid flag')
    if not all(-2200 <= x <= 2200 for x in v[11:14]) or abs(v[14]) > 25000:
        raise ValueError('Invalid IMU reading')
    return Sample(seq, millis, None if distance == -1 else distance / 1000,
                  left, right, tuple(v[5:8]), bool(v[8]), bool(v[9]), bool(v[10]),
                  tuple(x / 1000 for x in v[11:14]), v[14] / 100)

def drive_command(seq: int, left: int, right: int) -> bytes:
    if not 0 <= seq <= 0xffffffff or not all(0 <= x <= 110 for x in (left, right)):
        raise ValueError('Command exceeds forward-only demo limits')
    return f'D,{seq},{left},{right}\n'.encode('ascii')
