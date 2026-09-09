import numpy as np
from .base import Measurement,visible

class Radar:
    def __init__(self,cfg,rng):self.cfg=cfg;self.rng=rng
    def scan(self,p,body,origin,yaw,ego,ego_velocity,targets,t):
        out=[]
        for ident,bid,pos,vel,radius in targets:
            delta=pos-origin;distance=np.linalg.norm(delta)
            if bid==body or distance>self.cfg.radar_range or distance<.01:continue
            # Wide surround radar; terrain/buildings still occlude it.
            if not visible(p,body,bid,origin,pos) or self.rng.random()<.035:continue
            unit=delta/distance
            radial=np.dot(vel-ego_velocity,unit)+self.rng.normal(0,.13)
            # Coarse azimuth tracking supplies tangential velocity; Doppler supplies
            # the more accurate radial component. Both are noisy measurements.
            measured_velocity=vel+self.rng.normal(0,.45,3)
            measured_velocity+=(radial-np.dot(measured_velocity-ego_velocity,unit))*unit
            out.append(Measurement(ident,ego+delta+self.rng.normal(0,.6,3),measured_velocity,.36,4,t,'radar',.88,radius))
        return out
