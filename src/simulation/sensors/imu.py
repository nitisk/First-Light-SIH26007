import numpy as np

class IMU:
    def __init__(self,rng):self.rng=rng;self.previous=None;self.timestamp=0
    def update(self,velocity,orientation,angular,t):
        dt=max(.01,t-self.timestamp)
        acceleration=np.zeros(3) if self.previous is None else (np.array(velocity)-self.previous)/dt
        self.previous=np.array(velocity);self.timestamp=t
        return dict(acceleration=(acceleration+self.rng.normal(0,.06,3)).tolist(),
                    orientation=(np.array(orientation)+self.rng.normal(0,.004,3)).tolist(),
                    angular_rate=(np.array(angular)+self.rng.normal(0,.005,3)).tolist(),timestamp=t,variance=.0036)
