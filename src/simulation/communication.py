"""Two independent queued networks. Payloads are serialized copies, never object references."""
import heapq
import json
import numpy as np

class Network:
    def __init__(self,cfg,seed):
        self.cfg=cfg; self.rng=np.random.default_rng(seed); self.queue=[]; self.seq=0
        self.sent=0;self.dropped=0;self.received=0;self.latency_total=0;self.stale=0
    def send(self,source,destination,payload,t):
        self.sent+=1
        if self.rng.random()<self.cfg.packet_loss:self.dropped+=1;return
        data=json.loads(json.dumps(payload,allow_nan=False))
        if not isinstance(data,dict) or data.get('vehicle_id')!=source or data.get('timestamp')!=t:
            raise ValueError('Invalid network packet identity/timestamp')
        for key in ('position','velocity'):
            if key in data:
                array=np.asarray(data[key],dtype=float)
                if array.shape!=(3,) or not np.isfinite(array).all():raise ValueError(f'Invalid packet {key}')
        delay=self.rng.uniform(self.cfg.latency_min,self.cfg.latency_max)
        self.seq+=1
        heapq.heappush(self.queue,(t+delay,self.seq,destination,data))
    def deliver(self,t):
        delivered=[]
        while self.queue and self.queue[0][0]<=t:
            _,_,dest,data=heapq.heappop(self.queue)
            age=t-data['timestamp']
            if age>self.cfg.packet_ttl:self.stale+=1;continue
            self.received+=1;self.latency_total+=age
            delivered.append((dest,data))
        return delivered
    def metrics(self):
        return dict(sent=self.sent,dropped=self.dropped,received=self.received,stale=self.stale,
                    pending=len(self.queue),average_latency=self.latency_total/max(self.received,1))
