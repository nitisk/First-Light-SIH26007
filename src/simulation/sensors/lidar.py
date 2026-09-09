import numpy as np
from .base import Measurement

class Lidar:
    def __init__(self,cfg,rng):self.cfg=cfg;self.rng=rng;self.rays=[]
    def scan(self,p,body,origin,yaw,ego,targets,visibility,t):
        limit=min(self.cfg.lidar_range,visibility*1.35)
        angles=np.linspace(-np.pi,np.pi,48,endpoint=False)+yaw
        dirs=np.c_[np.cos(angles),np.sin(angles),np.zeros(len(angles))]
        starts=origin+dirs*4;ends=origin+dirs*limit
        hits=p.rayTestBatch(starts.tolist(),ends.tolist())
        catalogue={bid:(ident,radius) for ident,bid,_,_,radius in targets}
        out=[];seen=set();self.rays=[]
        for a,b,h in zip(starts,ends,hits):
            self.rays.append((a,np.array(h[3]) if h[0]>=0 else b,h[0]>=0))
            if h[0] not in catalogue or h[0]==body or h[0] in seen:continue
            if self.rng.random()<.08+.35*(1-visibility/70):continue
            seen.add(h[0]);ident,radius=catalogue[h[0]]
            surface=np.array(h[3])-origin;unit=surface/max(np.linalg.norm(surface),.001)
            # Surface-to-centre uncertainty is explicit; never replace a ray hit with truth.
            sigma=.35+radius*.45
            out.append(Measurement(ident,ego+surface+unit*radius*.65+self.rng.normal(0,.12,3),
                                   np.zeros(3),sigma**2,100,t,'lidar',.75,radius))
        return out
