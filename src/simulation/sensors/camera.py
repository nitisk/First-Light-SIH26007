import numpy as np
from .base import Measurement,visible

class Camera:
    def __init__(self,cfg,rng):self.cfg=cfg;self.rng=rng
    def scan(self,p,body,origin,yaw,ego,targets,visibility,t):
        out=[]; limit=min(self.cfg.camera_range,visibility)
        for ident,bid,pos,vel,radius in targets:
            delta=pos-origin;distance=np.linalg.norm(delta)
            angle=np.arctan2(delta[1],delta[0])-yaw
            if bid==body or distance>limit or np.cos(angle)<.5:continue
            if not visible(p,body,bid,origin,pos) or self.rng.random()>.93:continue
            sigma=.3+distance*.025+max(0,40-visibility)*.025
            bias=delta/max(distance,.001)*self.cfg.camera_bias
            out.append(Measurement(ident,ego+delta+bias+self.rng.normal(0,sigma,3),np.zeros(3),sigma**2,100,t,'camera',.8,radius))
        return out
