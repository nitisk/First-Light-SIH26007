"""Telemetry-only command centre. Never reads vehicles or issues collision decisions."""
class CommandCenter:
    def __init__(self):self.states={}
    def receive(self,packet):
        old=self.states.get(packet['vehicle_id'])
        if old is None or packet['timestamp']>old['timestamp']:self.states[packet['vehicle_id']]=packet
    def status(self,t):
        return {ident:dict(state,connection='ONLINE' if t-state['timestamp']<2 else 'STALE') for ident,state in self.states.items()}
