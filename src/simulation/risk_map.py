"""Per-vehicle map. Received observations preserve origin and observation time."""
from dataclasses import dataclass, asdict
import numpy as np

@dataclass
class Observation:
    cell: tuple
    position: list
    kind: str
    severity: float
    timestamp: float
    source: str

class RiskMap:
    def __init__(self,owner,ttl):self.owner=owner; self.ttl=ttl; self.cells={}; self.learned=0
    def observe(self,pos,kind,severity,t):
        cell=tuple(int(x//12) for x in pos)
        self.cells[(cell,kind)]=Observation(cell,list(pos),kind,float(severity),t,self.owner)
    def merge(self,reports,t):
        for report in reports:
            o=Observation(**report); o.cell=tuple(o.cell); key=(o.cell,o.kind)
            if t-o.timestamp>self.ttl:continue
            if key not in self.cells or self.cells[key].timestamp<o.timestamp:
                self.learned+=int(o.source!=self.owner and key not in self.cells)
                self.cells[key]=o
    def expire(self,t):self.cells={k:o for k,o in self.cells.items() if t-o.timestamp<=self.ttl}
    def reports(self):
        # Reserve packet space for environmental warnings so numerous rock
        # returns cannot starve fog reports out of the bounded payload.
        fog=sorted((o for o in self.cells.values() if o.kind=='fog'),key=lambda o:o.timestamp,reverse=True)[:6]
        obstacles=sorted((o for o in self.cells.values() if o.kind!='fog'),key=lambda o:o.timestamp,reverse=True)[:6]
        return [asdict(o) for o in fog+obstacles]
    def fog_ahead(self,route,s,t):
        future=[route.sample(s+d) for d in (12,24,36)]
        return any(o.kind=='fog' and o.source!=self.owner and o.severity>.55 and t-o.timestamp<self.ttl
                   and min(np.linalg.norm(np.array(o.position)-p) for p in future)<19 for o in self.cells.values())
