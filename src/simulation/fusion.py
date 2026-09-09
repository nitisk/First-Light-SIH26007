"""Variance-weighted, age-aware position/velocity fusion, including delivered V2V."""
from dataclasses import dataclass
import numpy as np
from sensors.base import Measurement

@dataclass
class Track:
    target: str
    position: np.ndarray
    velocity: np.ndarray
    variance: float
    confidence: float
    radius: float
    sources: tuple
    packet: dict | None

def fuse(measurements,inbox,t,cfg):
    all_m=list(measurements);packets={}
    for ident,packet in inbox.items():
        age=t-packet['timestamp']
        if age<0 or age>cfg.packet_ttl:continue
        packets[ident]=packet
        vel=np.array(packet['velocity'])
        all_m.append(Measurement(ident,np.array(packet['position'])+vel*age,vel,
                     packet['position_variance']+.5*age**2,.08+age*.4,t,'v2v',.9,3.8))
    groups={}
    for m in all_m:
        if m.valid() and t-m.timestamp<=cfg.packet_ttl:groups.setdefault(m.target,[]).append(m)
    tracks=[]
    for ident,group in groups.items():
        # Robust innovation gate: an overconfident biased optical return cannot
        # dominate two mutually consistent independent/cooperative observations.
        if len(group)>=3:
            centre=np.median(np.array([m.position for m in group]),axis=0)
            group=[m for m in group if np.linalg.norm(m.position-centre)<max(3,3*np.sqrt(m.variance))]
            if not group:continue
        weights=np.array([1/(m.variance+.2*(t-m.timestamp)**2) for m in group]);weights/=weights.sum()
        vw=np.array([1/m.velocity_variance for m in group]);vw/=vw.sum()
        pos=sum(w*(m.position+m.velocity*(t-m.timestamp)) for w,m in zip(weights,group))
        vel=sum(w*m.velocity for w,m in zip(vw,group))
        tracks.append(Track(ident,pos,vel,float(sum(w*m.variance for w,m in zip(weights,group))),
                     float(sum(w*m.confidence for w,m in zip(weights,group))),group[0].radius,
                     tuple(m.sensor for m in group),packets.get(ident)))
    return tracks
