"""Truth-based evaluation is separate from vehicle control."""
import math
import numpy as np

class Metrics:
    def __init__(self):
        self.collisions=set();self.near_misses=set();self.open_conflicts={};self.avoided=set();self.ttcs=[]
        self.min_separation=float('inf');self.risk_samples={k:0 for k in ('LOW','MEDIUM','HIGH','CRITICAL')}
    def update(self,p,vehicles,t,obstacles=()):
        ids={v.body:v.id for v in vehicles}
        hazard_ids={item[1]:item[0] for item in obstacles}
        for contact in p.getContactPoints():
            a,b=contact[1:3]
            if a in ids and b in ids and a!=b:self.collisions.add(tuple(sorted((ids[a],ids[b]))))
            elif a in ids and b in hazard_ids:self.collisions.add(tuple(sorted((ids[a],hazard_ids[b]))))
            elif b in ids and a in hazard_ids:self.collisions.add(tuple(sorted((ids[b],hazard_ids[a]))))
        for i,a in enumerate(vehicles):
            self.risk_samples[a.decision.risk]+=1
            if math.isfinite(a.decision.ttc):self.ttcs.append(a.decision.ttc)
            if a.decision.risk in ('HIGH','CRITICAL') and a.decision.target.startswith('V'):
                key=tuple(sorted((a.id,a.decision.target)))
                self.open_conflicts[key]=t
            for b in vehicles[i+1:]:
                distance=float(np.linalg.norm(a.position-b.position));self.min_separation=min(self.min_separation,distance)
                if distance<6.5:self.near_misses.add(tuple(sorted((a.id,b.id))))
        for key,last in list(self.open_conflicts.items()):
            if t-last>5:
                if key not in self.collisions:self.avoided.add(key)
                del self.open_conflicts[key]
    def summary(self,vehicles,v2v,uplink,t):
        return dict(vehicles=len(vehicles),routes_completed=sum(v.parked for v in vehicles),vehicles_parked=sum(v.parked for v in vehicles),
                    collisions=len(self.collisions),collision_pairs=sorted(self.collisions),avoided_conflicts=len(self.avoided),
                    near_misses=len(self.near_misses),minimum_centre_separation=self.min_separation,
                    minimum_ttc=min(self.ttcs) if self.ttcs else None,average_finite_ttc=float(np.mean(self.ttcs)) if self.ttcs else None,
                    braking_events=sum(v.response.events for v in vehicles),v2v=v2v.metrics(),command_uplink=uplink.metrics(),
                    sensor_detections={name:sum(v.perception.detections[name] for v in vehicles) for name in ('camera','lidar','radar')},
                    optical_degraded_scans=sum(v.perception.degraded for v in vehicles),
                    shared_fog_warning_events=sum(v.fog_warning_events for v in vehicles),
                    risk_samples=self.risk_samples,route_completion={v.id:round(v.progress/v.route.length,4) for v in vehicles},
                    parking_errors={v.id:float(np.linalg.norm(v.position[:2]-v.route.points[-1,:2])) for v in vehicles},simulation_duration=round(t,2))
