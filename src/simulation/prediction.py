"""No PyBullet or vehicle registry here: only estimates, own route, delivered packets."""
from dataclasses import dataclass
import numpy as np

@dataclass
class Decision:
    risk: str = 'LOW'
    ttc: float = float('inf')
    distance: float = float('inf')
    target: str = ''
    reason: str = 'Road clear'
    yield_required: bool = False

def predict(ident,position,velocity,route,progress,tracks,cfg):
    result=Decision(); rank={'LOW':0,'MEDIUM':1,'HIGH':2,'CRITICAL':3}
    speed=np.linalg.norm(velocity);direction=route.direction(progress)
    junction,jdistance=route.next_junction(progress)
    for track in tracks:
        r=track.position-position;distance=np.linalg.norm(r);v=track.velocity-velocity
        closing=-np.dot(r,v)/max(distance,.001)
        safe=cfg.safe_distance if track.target.startswith('V') else 3.0+track.radius
        clearance=distance-safe
        ttc=max(0,clearance)/closing if closing>.05 else float('inf')
        cpa_t=float(np.clip(-np.dot(r,v)/max(np.dot(v,v),.001),0,cfg.horizon))
        cpa=np.linalg.norm(r+v*cpa_t)
        ahead=np.dot(r,direction)>0 and np.linalg.norm(r-np.dot(r,direction)*direction)<5
        conflict=cpa<safe and closing>.05
        yielding=False;reason='Projected closest approach'
        packet=track.packet
        if packet:
            # Compare road-aware trajectories, not proximity on unrelated benches.
            theirs=np.asarray(packet['trajectory']);ours=np.asarray(route.future(progress,max(speed,1)))
            overlap=np.linalg.norm(ours-theirs,axis=1)
            conflict=conflict or (float(overlap.min())<cfg.safe_distance and closing>.05)
            other_j,other_d=packet['junction'],packet['junction_distance']
            if junction and junction==other_j and 0<jdistance<36 and other_d is not None and -7<other_d<36:
                eta=jdistance/max(speed,1);other_eta=max(other_d,0)/max(packet['speed'],1)
                # Road-approach priority cannot form the ID/queue cycles that occur
                # when a high-ID leader blocks its low-ID follower at a merge.
                following_behind=packet['route_segment']==route.segment(progress) and other_d>jdistance+3
                other_priority=(packet['route_segment'],track.target)
                own_priority=(route.segment(progress),ident)
                if abs(eta-other_eta)<12 and other_priority<own_priority and not following_behind:
                    yielding=True;conflict=True;reason=f'Yield {junction} to {track.target}'
                    ttc=min(ttc,max(0,jdistance-9)/max(speed,.5))
        hold=ahead and distance<safe+1
        risk='LOW'
        if conflict or hold:
            risk='CRITICAL' if hold or ttc<cfg.critical_ttc else 'HIGH' if ttc<cfg.high_ttc else 'MEDIUM' if ttc<cfg.medium_ttc or yielding else 'LOW'
        # Following traffic with matching velocity still needs a distance latch.
        if rank[risk]>0 and (rank[risk]>rank[result.risk] or (risk==result.risk and ttc<result.ttc)):
            result=Decision(risk,ttc,distance,track.target,reason,yielding)
    return result
