"""Bounded PyBullet debug HUD plus an optional standard-library Tk dashboard."""
import math
import numpy as np

RISK_COLORS={'LOW':(.25,.9,.55),'MEDIUM':(1,.85,.18),'HIGH':(1,.45,.1),'CRITICAL':(1,.15,.15)}

class HUD:
    def __init__(self,p,cfg):
        self.p=p;self.cfg=cfg;self.lines={};self.selected=0;self.root=None;self.paused=False;self.view=0
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,1)
        p.resetDebugVisualizerCamera(215,43,-49,[5,6,10])
        try:
            import tkinter as tk
            self.root=tk.Tk();self.root.title('FIRST LIGHT | Fleet command');self.root.geometry('670x660')
            self.root.configure(bg='#101e26');self.root.protocol('WM_DELETE_WINDOW',self.close_dashboard)
            self.header=tk.Label(self.root,text='FIRST LIGHT',font=('Segoe UI',26,'bold'),bg='#101e26',fg='#5fe3cf');self.header.pack(anchor='w',padx=22,pady=(15,0))
            tk.Label(self.root,text='PREDICTIVE FOG-ADAPTIVE COOPERATIVE MINE SAFETY',font=('Segoe UI',9),bg='#101e26',fg='#a0b7c2').pack(anchor='w',padx=22)
            self.text=tk.Label(self.root,justify='left',anchor='nw',font=('Consolas',11),bg='#101e26',fg='#e4eef0',padx=22,pady=22);self.text.pack(fill='both',expand=True)
            tk.Label(self.root,text='PyBullet: 1–6 select truck  |  C camera  |  SPACE pause  |  ESC exit',font=('Segoe UI',10),bg='#172a34',fg='#94b7c2',pady=12).pack(fill='x')
        except (ImportError,Exception) as exc:
            print('Tk dashboard unavailable; using 3D HUD:',exc);self.root=None
    def close_dashboard(self):
        if self.root:self.root.destroy();self.root=None
    def text3d(self,key,text,pos,color=(.9,.95,1),size=1.2):
        self.lines[key]=self.p.addUserDebugText(text,pos,color,size,replaceItemUniqueId=self.lines.get(key,-1))
    def line(self,key,a,b,color):
        self.lines[key]=self.p.addUserDebugLine(list(a),list(b),color,1,replaceItemUniqueId=self.lines.get(key,-1))
    def keys(self,vehicles):
        events=self.p.getKeyboardEvents()
        for i in range(len(vehicles)):
            if events.get(ord(str(i+1)),0)&self.p.KEY_WAS_TRIGGERED:self.selected=i
        if events.get(ord(' '),0)&self.p.KEY_WAS_TRIGGERED:self.paused=not self.paused
        if events.get(ord('c'),0)&self.p.KEY_WAS_TRIGGERED:
            self.view=(self.view+1)%3
            if self.view==0:self.p.resetDebugVisualizerCamera(215,43,-49,[5,6,10])
            elif self.view==2:self.p.resetDebugVisualizerCamera(82,25,-45,[108,46,18])
        if self.view==1:self.p.resetDebugVisualizerCamera(27,vehicles[self.selected].yaw*180/math.pi-90,-23,vehicles[self.selected].position)
        return bool(events.get(27,0)&self.p.KEY_WAS_TRIGGERED)
    def update(self,centre,vehicles,v2v,metrics,t):
        states=centre.status(t);parked=sum(s['parking']=='PARKED' for s in states.values())
        selected=vehicles[self.selected];s=states.get(selected.id)
        rows=['FLEET TELEMETRY                         T + %6.1f s'%t,
              f'{parked}/{len(vehicles)} PARKED   |   {len(metrics.collisions)} CONTACTS   |   {len(metrics.avoided)} CONFLICTS RESOLVED','',
              'ID  LINK    SPEED   VISIBILITY  RISK      DESTINATION']
        for ident,state in sorted(states.items()):
            rows.append(f"{ident:3} {state['connection']:7} {state['speed']:4.1f}   {state['visibility']:5.1f} m    {state['risk']:8}  {state['parking']}")
        rows.extend(['',f'V2V  RX {v2v.received}  LOST {v2v.dropped}  AVG LATENCY {1000*v2v.latency_total/max(1,v2v.received):.0f} ms'])
        approaching=sum(s['junction_distance'] is not None and 0<s['junction_distance']<36 for s in states.values())
        dangerous=sum(s['risk'] in ('HIGH','CRITICAL') for s in states.values())
        rows.append(f'ACTIVE {len(states)-parked}   APPROACHING {approaching}   ALERTS {dangerous}')
        if s:
            rows+=['','SELECTED VEHICLE / '+selected.id,'─'*55,
                   f"POSITION  {s['position'][0]:6.1f}, {s['position'][1]:6.1f}   ELEV {s['elevation']:.1f} m",
                   f"SPEED     {s['speed']:.2f} m/s   TARGET {s['target_speed']:.2f} m/s",
                   f"ROUTE     {s['route_segment']}   {s['route_progress']*100:.1f}%",
                   f"CAMERA    {s['sensors'].get('camera','warming up')}",f"LIDAR     {s['sensors'].get('lidar','warming up')}",
                   f"RADAR     {s['sensors'].get('radar','warming up')}",f"GNSS AGE  {s['sensors']['gnss_age']:.2f}s   V2V PEERS {s['v2v_peers']}",
                   f"TTC       {s['ttc'] if s['ttc'] is not None else 'infinite'}",f"SAFETY    {s['risk']} / {s['reason']}",
                   f"THREAT    {round(s['threat_distance'],1) if s['threat_distance'] is not None else 'none'} m   COMMAND {s['command_connection']}",
                   'FOG AHEAD / V2V REPORT' if s['fog_ahead'] else 'LOCAL RISK MAP ACTIVE']
        if self.root:
            try:self.text.configure(text='\n'.join(rows));self.root.update()
            except Exception:self.root=None
        else:
            # Embedded Python can lack Tk. Camera-relative debug text still gives
            # the complete HUD, rather than reducing the portable demo to labels.
            camera=self.p.getDebugVisualizerCamera()
            inverse=np.linalg.inv(np.array(camera[2]).reshape(4,4,order='F'))
            projection=np.array(camera[3]).reshape(4,4,order='F')
            depth=10.;half_x=depth/projection[0,0];half_y=depth/projection[1,1]
            for i,row in enumerate(rows):
                local=np.array([-half_x+.22,half_y-.22-i*.18,-depth,1])
                anchor=(inverse@local)[:3]
                self.text3d(f'overlay{i}',row,anchor,(.72,.95,.93),.85)
        self.text3d('title',f'FIRST LIGHT   |   {t:.1f}s   |   PARKED {parked}/{len(vehicles)}   |   COLLISIONS {len(metrics.collisions)}',[75,76,34],(.3,.95,.9),1.5)
        for v in vehicles:
            state=states.get(v.id)
            if state:self.text3d(v.id,f"{v.id} | {state['risk']} | {state['speed']:.1f} m/s | {state['parking']}",v.position+[0,0,5],RISK_COLORS[state['risk']])
        if self.cfg.visualization:
            for i,(a,b,hit) in enumerate(selected.perception.lidar.rays):
                if not self.cfg.sensor_debug and i%3:continue
                self.line(f'ray{i}',a,b,(.95,.3,.15) if hit else (.15,.6,.64))
            for i,angle in enumerate((-.52,.52,-1.05,1.05)):
                length=min(selected.visibility,30) if i<2 else 38
                end=selected.position+[math.cos(selected.yaw+angle)*length,math.sin(selected.yaw+angle)*length,0]
                self.line(f'cone{i}',selected.position,end,(.35,.8,.95) if i<2 else (.78,.54,.95))
            for i,other in enumerate(vehicles):
                packet=selected.inbox.get(other.id)
                end=np.array(packet['position']) if packet else selected.position
                self.line(f'link{i}',selected.position+[0,0,3],end+[0,0,3],(.3,.8,.55))
