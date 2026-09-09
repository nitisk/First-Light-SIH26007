import numpy as np

class Response:
    def __init__(self):self.command=0.;self.braking=False;self.events=0
    def update(self,decision,visibility,fog_ahead,remaining,speed,cfg,dt):
        target=cfg.cruise*float(np.clip(visibility/55,.35,1))
        if fog_ahead:target=min(target,2.5)
        target=min(target,np.sqrt(max(0,2*1.2*(remaining-.4))))
        factor={'LOW':1,'MEDIUM':.55,'HIGH':.15,'CRITICAL':0}[decision.risk]
        target*=factor
        if decision.yield_required and decision.ttc<4:target=0
        braking=bool(target<speed-.25)
        if braking and not self.braking:self.events+=1
        self.braking=braking
        rate=cfg.braking if target<self.command else cfg.acceleration
        self.command+=float(np.clip(target-self.command,-rate*dt,rate*dt))
        return self.command
