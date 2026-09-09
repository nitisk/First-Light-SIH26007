from sensors.camera import Camera
from sensors.lidar import Lidar
from sensors.radar import Radar
from sensors.gnss import GNSS,SpeedSensor
from sensors.imu import IMU

class Perception:
    def __init__(self,cfg,rng):
        self.camera=Camera(cfg,rng);self.lidar=Lidar(cfg,rng);self.radar=Radar(cfg,rng)
        self.gnss=GNSS(rng,cfg.gnss_period);self.speed=SpeedSensor(rng);self.imu=IMU(rng)
        self.status={};self.detections={'camera':0,'lidar':0,'radar':0};self.degraded=0
    def scan(self,p,body,pos,yaw,localization,targets,visibility,t):
        groups=[self.camera.scan(p,body,pos,yaw,localization.position,targets,visibility,t),
                self.lidar.scan(p,body,pos,yaw,localization.position,targets,visibility,t),
                self.radar.scan(p,body,pos,yaw,localization.position,localization.velocity,targets,t)]
        for name,group in zip(('camera','lidar','radar'),groups):
            self.detections[name]+=len(group)
            self.status[name]=f'{len(group)} hits'+(' / DEGRADED' if visibility<25 and name!='radar' else '')
        self.degraded+=int(visibility<25)
        return [m for group in groups for m in group if m.valid()]
