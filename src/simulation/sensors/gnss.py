import numpy as np

class GNSS:
    def __init__(self,rng,period):self.rng=rng;self.period=period;self.timestamp=-1e9;self.position=None;self.variance=.12**2
    def update(self,truth,t):
        if t-self.timestamp>=self.period-1e-8:
            self.position=np.array(truth)+self.rng.normal(0,.12,3);self.timestamp=t
        return self.position.copy(),self.variance,self.timestamp

class SpeedSensor:
    def __init__(self,rng):self.rng=rng;self.variance=.04**2;self.timestamp=0;self.value=0
    def update(self,speed,t):
        self.value=max(0,float(speed+self.rng.normal(0,.04)));self.timestamp=t
        return self.value
