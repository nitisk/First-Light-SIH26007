"""Dead reckoning between GNSS updates; safety consumes this estimate."""
import numpy as np

class Positioning:
    def __init__(self):self.position=None;self.velocity=np.zeros(3);self.variance=1;self.timestamp=0
    def update(self,gnss,speed,imu,t):
        pos,var,stamp=gnss
        roll,pitch,yaw=imu['orientation']
        self.velocity=speed*np.array([np.cos(yaw)*np.cos(pitch),np.sin(yaw)*np.cos(pitch),-np.sin(pitch)])
        self.position=pos+self.velocity*(t-stamp)
        self.variance=var+.08*(t-stamp)**2;self.timestamp=t
