"""All distances metres, time seconds, speeds m/s. Reproducible demo defaults."""
from dataclasses import dataclass

@dataclass
class Config:
    seed: int = 42
    vehicles: int = 4
    scenario: str = 'demo'
    dt: float = .05
    physics_steps: int = 5
    duration: float = 360
    cruise: float = 5.0
    acceleration: float = 1.1
    braking: float = 3.5
    safe_distance: float = 7.5
    critical_ttc: float = 2.2
    high_ttc: float = 4.0
    medium_ttc: float = 7.0
    horizon: float = 8.0
    camera_range: float = 45
    camera_bias: float = 0
    lidar_range: float = 48
    radar_range: float = 85
    gnss_period: float = .25
    sensor_period: float = .2
    packet_period: float = .25
    packet_loss: float = .08
    latency_min: float = .08
    latency_max: float = .3
    packet_ttl: float = 1.5
    map_ttl: float = 25
    headless: bool = False
    visualization: bool = True
    sensor_debug: bool = False
    network_debug: bool = False
