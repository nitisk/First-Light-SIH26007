"""Drifting, pulsating Gaussian banks; visual wisps are independent of physics."""
import numpy as np

class Fog:
    def __init__(self,seed):
        self.phase=np.random.default_rng(seed).uniform(0,6.28,3)
        self.centres=np.array([[-40,-12,9],[30,12,18],[78,20,18]],float)
        self.density=np.ones(3)
    def update(self,t):
        self.centres[:,0]=np.array([-40,30,78])+8*np.sin(t/35+self.phase)
        self.centres[:,1]=np.array([-12,12,20])+9*np.cos(t/43+self.phase)
        self.density=.65+.3*np.sin(t/29+self.phase)**2
    def density_at(self,pos):
        d=(self.centres-np.asarray(pos))/np.array([24,21,10])
        return float(np.clip(np.sum(self.density*np.exp(-.5*np.sum(d*d,axis=1))),0,1))
    def visibility(self,pos):return 70-61*self.density_at(pos)
    @staticmethod
    def label(visibility):return 'DENSE' if visibility<22 else 'MODERATE' if visibility<45 else 'CLEAR'
