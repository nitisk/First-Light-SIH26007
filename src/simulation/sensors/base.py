from dataclasses import dataclass
import numpy as np

@dataclass
class Measurement:
    target: str
    position: np.ndarray
    velocity: np.ndarray
    variance: float
    velocity_variance: float
    timestamp: float
    sensor: str
    confidence: float
    radius: float = 3.8
    def valid(self):
        return (np.isfinite(self.position).all() and np.isfinite(self.velocity).all()
                and self.variance>0 and self.velocity_variance>0 and 0<=self.confidence<=1)

def visible(p,body,target,origin,point):
    # Place ray origin beyond own chassis to avoid self occlusion.
    delta=np.array(point)-origin; length=np.linalg.norm(delta)
    if length<4:return True
    start=origin+delta/length*4
    hit=p.rayTest(start.tolist(),list(point))[0][0]
    return hit in (-1,target,body)
