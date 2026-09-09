"""Measured range/TTC, speed feedback and line following for a small rover.

Full-size simulation constants are deliberately not applied to a tabletop robot.
Inputs are sensor samples; this module has no transport or simulator access.
"""
from dataclasses import dataclass, fields
import json
import math
from pathlib import Path
from .protocol import Sample

@dataclass
class Config:
    vehicle_id: str = 'V1'
    zone: str = 'demo-road'
    wheel_diameter_m: float = 0.065
    encoder_pulses_per_revolution: int = 20
    cruise_mps: float = 0.18
    fog_mps: float = 0.07
    stop_distance_m: float = 0.20
    release_distance_m: float = 0.30
    critical_ttc_s: float = 0.8
    warning_ttc_s: float = 2.5
    acceleration_mps2: float = 0.15
    deceleration_mps2: float = 0.4
    line_threshold: int = 500
    line_black_high: bool = True
    line_turn_gain: float = 0.5
    max_pwm: int = 110
    pwm_per_mps: float = 450.0
    speed_kp: float = 100.0
    max_tilt_deg: float = 25
    sensor_timeout_s: float = 0.35
    stall_timeout_s: float = 1.5
    peer_timeout_s: float = 1.0

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if f.name in ('vehicle_id', 'zone'):
                if not isinstance(value, str) or not 1 <= len(value) <= 40:
                    raise ValueError(f'Invalid {f.name}')
            elif f.name == 'line_black_high':
                if type(value) is not bool:
                    raise ValueError('line_black_high must be boolean')
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{f.name} must be finite and positive')
        for key in ('encoder_pulses_per_revolution', 'line_threshold', 'max_pwm'):
            if type(getattr(self, key)) is not int:
                raise ValueError(f'{key} must be an integer')
        if not (self.stop_distance_m >= .12 and self.release_distance_m > self.stop_distance_m):
            raise ValueError('Range thresholds must exceed firmware guard distance')
        if not (self.fog_mps <= self.cruise_mps <= .25 and self.max_pwm <= 110):
            raise ValueError('Tabletop speed/PWM limits exceeded')
        if not (self.line_threshold < 1023 and self.line_turn_gain <= 1 and self.max_tilt_deg < 90):
            raise ValueError('Invalid line/tilt limits')
        if not (self.critical_ttc_s < self.warning_ttc_s and self.sensor_timeout_s <= .5):
            raise ValueError('Invalid time limits')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding='utf-8')))

@dataclass
class Output:
    state: str
    reason: str
    left_pwm: int = 0
    right_pwm: int = 0
    speed_mps: float = 0.0
    target_mps: float = 0.0
    ttc_s: float | None = None

class Controller:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.previous = None
        self.last_rx = None
        self.target = 0.0
        self.closing = 0.0
        self.blocked = False
        self.fault = ''
        self.parked = False
        self.stall_since = None
        self.last_pwm = (0, 0)

    def stop(self, state, reason, speed=0.0, ttc=None):
        self.target = 0.0
        self.last_pwm = (0, 0)
        return Output(state, reason, speed_mps=speed, ttc_s=ttc)

    def update(self, sample: Sample | None, now: float, armed=False, peer_warning=False):
        c = self.cfg
        if sample is None:
            if self.last_rx is not None and now - self.last_rx > c.sensor_timeout_s:
                self.fault = 'Telemetry timeout; restart to rearm'
            return self.stop('FAULT' if self.fault else 'WAITING', self.fault or 'Waiting for sensors')
        speed = 0.0
        dt = .1
        if self.previous is not None:
            seq_delta = (sample.seq - self.previous.seq) & 0xffffffff
            elapsed = ((sample.millis - self.previous.millis) & 0xffffffff) / 1000
            if seq_delta == 0:
                return self.update(None, now, armed, peer_warning)
            if seq_delta > 0x7fffffff or not 0 < elapsed <= .5:
                self.fault = 'Controller reset or invalid sample timing; restart to rearm'
            else:
                dt = elapsed
                deltas = [(sample.left_ticks - self.previous.left_ticks) & 0xffffffff,
                          (sample.right_ticks - self.previous.right_ticks) & 0xffffffff]
                wheel_speeds = [d * math.pi * c.wheel_diameter_m / c.encoder_pulses_per_revolution / dt for d in deltas]
                speed = sum(wheel_speeds) / 2
                if max(wheel_speeds) > 1.5:
                    self.fault = 'Encoder speed outside tabletop range'
                if max(self.last_pwm) >= 40 and min(wheel_speeds) < .005:
                    self.stall_since = now if self.stall_since is None else self.stall_since
                    if now - self.stall_since >= c.stall_timeout_s:
                        self.fault = 'Wheel stalled or encoder missing; restart to rearm'
                else:
                    self.stall_since = None
                if sample.distance_m is not None and self.previous.distance_m is not None:
                    measured = (self.previous.distance_m - sample.distance_m) / dt
                    self.closing = .5 * self.closing + .5 * measured
                else:
                    self.closing = 0.0
        if self.last_rx is not None and now - self.last_rx > c.sensor_timeout_s:
            self.fault = 'Telemetry gap; restart to rearm'
        self.previous, self.last_rx = sample, now
        if sample.estop:
            self.fault = 'Emergency stop; reset Uno and restart controller'
        if not sample.imu_ok:
            self.fault = 'IMU unavailable; check wiring and restart'
        ax, ay, az = sample.accel_g
        tilt = math.degrees(math.atan2(math.hypot(ax, ay), az))
        if not .5 < math.sqrt(ax*ax + ay*ay + az*az) < 1.8 or tilt > c.max_tilt_deg:
            self.fault = 'Tilt or acceleration limit; restart to rearm'
        if self.fault:
            return self.stop('FAULT', self.fault, speed)
        if not armed:
            return self.stop('DISARMED', 'Monitoring only; --arm enables motors', speed)
        if self.parked:
            return self.stop('PARKED', 'Destination marker reached', speed)
        black = [v >= c.line_threshold if c.line_black_high else v <= c.line_threshold for v in sample.line]
        if all(black):
            self.parked = True
            return self.stop('PARKED', 'Destination marker reached', speed)
        if not any(black):
            return self.stop('STOP', 'Guide line lost', speed)
        if sample.distance_m is None:
            return self.stop('STOP', 'No valid ultrasonic echo', speed)
        distance = sample.distance_m
        ttc = max(0, distance-c.stop_distance_m) / self.closing if self.closing > .02 else None
        braking_distance = c.stop_distance_m + speed**2 / (2*c.deceleration_mps2)
        if distance <= braking_distance or (ttc is not None and ttc < c.critical_ttc_s):
            self.blocked = True
        if self.blocked:
            if distance <= max(c.release_distance_m, braking_distance) or (ttc is not None and ttc < c.warning_ttc_s):
                return self.stop('STOP', 'Obstacle / TTC stop with distance hold', speed, ttc)
            self.blocked = False
        wanted = c.fog_mps if sample.fog or peer_warning else c.cruise_mps
        caution = sample.fog or peer_warning or (ttc is not None and ttc < c.warning_ttc_s)
        if caution:
            wanted = min(wanted, c.fog_mps)
        rate = c.acceleration_mps2 if wanted > self.target else c.deceleration_mps2
        self.target += max(-rate*dt, min(rate*dt, wanted-self.target))
        pwm = max(0, min(c.max_pwm, c.pwm_per_mps*self.target + c.speed_kp*(self.target-speed)))
        turn = (int(black[2])-int(black[0])) * c.line_turn_gain
        left = round(min(c.max_pwm, pwm*(1+turn)))
        right = round(min(c.max_pwm, pwm*(1-turn)))
        self.last_pwm = (left, right)
        return Output('CAUTION' if caution else 'CRUISE',
                      'Fog/shared warning/TTC caution' if caution else 'Following guide line',
                      left, right, speed, self.target, ttc)
