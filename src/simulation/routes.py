"""Directed, surveyed road graph; smooth sampled centre lines and arc-length routes."""
from dataclasses import dataclass
import heapq
import numpy as np

def smooth(points, spacing=1.0):
    p = np.asarray(points, float)
    out = []
    for i in range(len(p)-1):
        a,b,c,d = p[max(i-1,0)],p[i],p[i+1],p[min(i+2,len(p)-1)]
        for t in np.linspace(0,1,max(4,int(np.linalg.norm(c-b)/spacing)),endpoint=False):
            out.append(.5*((2*b)+(-a+c)*t+(2*a-5*b+4*c-d)*t*t+(-a+3*b-3*c+d)*t*t*t))
    return np.vstack([out,p[-1]])

@dataclass
class Edge:
    start: str
    end: str
    points: np.ndarray
    width: float = 10
    @property
    def distance(self): return float(np.linalg.norm(np.diff(self.points,axis=0),axis=1).sum())
    @property
    def elevation(self): return (float(self.points[0,2]),float(self.points[-1,2]))

class Route:
    def __init__(self, points, segments, junctions):
        self.points = np.asarray(points)
        self.arc = np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.points,axis=0),axis=1))]
        self.length = float(self.arc[-1])
        self.segments = segments
        self.junctions = junctions
    def sample(self,s):
        return np.array([np.interp(np.clip(s,0,self.length),self.arc,self.points[:,i]) for i in range(3)])
    def direction(self,s):
        d=self.sample(min(s+1,self.length))-self.sample(max(0,s-1))
        return d/max(np.linalg.norm(d),1e-9)
    def project(self,p,previous=0):
        ix=np.where((self.arc>=max(0,previous-4)) & (self.arc<=previous+18))[0]
        if not len(ix): return previous
        k=ix[np.argmin(np.linalg.norm(self.points[ix]-p,axis=1))]
        return max(previous,float(self.arc[k]))
    def segment(self,s):
        return next((name for end,name in self.segments if s<=end),self.segments[-1][1])
    def next_junction(self,s):
        return next(((name,at-s) for name,at in self.junctions if at-s>-8),('',float('inf')))
    def future(self,s,speed,horizon=8):
        samples=np.clip(s+speed*np.arange(0,horizon+.1,.5),0,self.length)
        return np.column_stack([np.interp(samples,self.arc,self.points[:,i]) for i in range(3)]).tolist()

class RoadNetwork:
    def __init__(self, count):
        self.nodes={'LOW':(-95,-52,0),'MID':(-85,44,9),'HIGH':(12,76,18),
                    'WEST':(-126,0,9),'J':(-28,0,9),'K':(35,0,18),'GATE':(76,22,18)}
        self.edges=[]
        def add(a,b,via,width=10):
            self.edges.append(Edge(a,b,smooth([self.nodes[a],*via,self.nodes[b]]),width))
        add('LOW','J',[(-82,-57,0),(-60,-48,2),(-42,-25,6)])
        add('MID','J',[(-72,43,9),(-54,30,9),(-38,14,9)])
        add('HIGH','K',[(33,70,18),(52,50,18),(51,24,18)])
        add('WEST','J',[(-111,2,9),(-88,-9,9),(-63,-11,9),(-42,-4,9)])
        add('J','K',[(-13,0,10),(2,0,13),(18,0,16)],12)
        add('K','GATE',[(49,1,18),(63,8,18)],12)
        self.slots=[]
        for i in range(count):
            x=84+i*9
            name=f'SLOT{i+1}'
            self.nodes[name]=(x,51,18)
            self.slots.append(np.array(self.nodes[name],float))
            add('GATE',name,[(83,24,18),(x+2,29,18),(x,40,18)],7)
        self.validate()
    def shortest(self,start,end):
        queue=[(0,start,[])]
        visited=set()
        while queue:
            cost,node,path=heapq.heappop(queue)
            if node==end:return path
            if node in visited:continue
            visited.add(node)
            for idx,e in enumerate(self.edges):
                if e.start==node:heapq.heappush(queue,(cost+e.distance,e.end,path+[idx]))
        raise ValueError(f'No road from {start} to {end}')
    def route(self,i):
        start=['LOW','MID','HIGH','WEST'][i%4]
        edges=[self.edges[k] for k in self.shortest(start,f'SLOT{i+1}')]
        points=[]; segments=[]; junctions=[]; length=0
        for e in edges:
            points.extend(e.points if not points else e.points[1:])
            length+=e.distance
            segments.append((length,e.start+'>'+e.end))
            if e.end in ('J','K','GATE'):junctions.append((e.end,length))
        return Route(points,segments,junctions)
    def validate(self):
        for e in self.edges:
            assert e.distance>0 and e.width>=7 and np.isfinite(e.points).all()
            assert np.allclose(e.points[0],self.nodes[e.start])
            assert np.allclose(e.points[-1],self.nodes[e.end])
        assert sum(e.end=='J' for e in self.edges)>=3
