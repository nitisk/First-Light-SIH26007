"""First Light entry point. Run `python main.py --help`."""
import argparse
from dataclasses import fields
import json
from pathlib import Path
import time
import numpy as np
import pybullet as p
from config import Config
from routes import RoadNetwork
from environment import Environment
from fog import Fog
from vehicle import Vehicle
from communication import Network
from command_center import CommandCenter
from metrics import Metrics

class Simulation:
    def __init__(self,cfg):
        self.cfg=cfg;self.client=p.connect(p.DIRECT if cfg.headless else p.GUI,options='--background_color_red=0.55 --background_color_green=0.65 --background_color_blue=0.71')
        p.setGravity(0,0,-9.81);p.setTimeStep(cfg.dt/cfg.physics_steps)
        p.setPhysicsEngineParameter(numSolverIterations=30,deterministicOverlappingPairs=1)
        if not cfg.headless:p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
        self.network=RoadNetwork(cfg.vehicles);self.environment=Environment(p,self.network,cfg);self.fog=Fog(cfg.seed)
        self.vehicles=[Vehicle(p,i,self.network.route(i),cfg) for i in range(cfg.vehicles)]
        self.v2v=Network(cfg,cfg.seed+400);self.uplink=Network(cfg,cfg.seed+800)
        self.acknowledgements=Network(cfg,cfg.seed+1200)
        self.centre=CommandCenter();self.metrics=Metrics();self.t=0.;self.last_packet=-1e9;self.hud=None
        if not cfg.headless:
            from ui import HUD
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1);self.hud=HUD(p,cfg)
    def step(self):
        cfg=self.cfg;t=self.t
        self.fog.update(t)
        registry={v.id:v for v in self.vehicles}
        for dest,packet in self.v2v.deliver(t):registry[dest].receive(packet,t)
        for dest,_ in self.acknowledgements.deliver(t):registry[dest].last_command_ack=t
        for _,packet in self.uplink.deliver(t):
            self.centre.receive(packet)
            self.acknowledgements.send('COMMAND',packet['vehicle_id'],dict(vehicle_id='COMMAND',timestamp=t),t)
        for v in self.vehicles:v.truth()
        targets=self.environment.targets+[(v.id,v.body,v.position.copy(),v.velocity.copy(),3.8) for v in self.vehicles]
        for v in self.vehicles:v.sense_decide(targets,self.fog,t)
        if t-self.last_packet>=cfg.packet_period-1e-8:
            self.last_packet=t
            for v in self.vehicles:
                packet=v.packet(t)
                for other in self.vehicles:
                    if other.id!=v.id:self.v2v.send(v.id,other.id,packet,t)
                self.uplink.send(v.id,'COMMAND',v.telemetry(t),t)
        for _ in range(cfg.physics_steps):
            for v in self.vehicles:v.actuate()
            p.stepSimulation()
        for v in self.vehicles:v.truth()
        self.metrics.update(p,self.vehicles,t,self.environment.targets)
        self.t+=cfg.dt
        if self.hud and int(round(self.t/cfg.dt))%4==0:
            self.environment.update_fog(self.fog,t);self.hud.update(self.centre,self.vehicles,self.v2v,self.metrics,t)
        if cfg.network_debug and int(round(self.t/cfg.dt))%200==0:print(f'{self.t:.1f}s V2V {self.v2v.metrics()}')
    def run(self,snapshot=None):
        finish=None
        while self.t<self.cfg.duration:
            start=time.perf_counter()
            if self.hud:
                if not p.isConnected() or self.hud.keys(self.vehicles):break
                if self.hud.paused:time.sleep(.05);continue
            self.step()
            if all(v.parked for v in self.vehicles):
                if finish is None:finish=self.t
                if self.t-finish>3:break
            if self.hud:time.sleep(max(0,self.cfg.dt-(time.perf_counter()-start)))
        result=self.metrics.summary(self.vehicles,self.v2v,self.uplink,self.t)
        if snapshot:self.snapshot(snapshot)
        return result
    def snapshot(self,path):
        from rendering import export_frame
        export_frame(self,path)
        if all(v.parked for v in self.vehicles):
            export_frame(self,Path(path).with_name(Path(path).stem+'-parking.png'),parking=True)
    def close(self):
        if self.hud:self.hud.close_dashboard()
        if p.isConnected():p.disconnect()

def arguments():
    parser=argparse.ArgumentParser(description='FIRST LIGHT — cooperative fog-adaptive autonomous mine fleet')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--vehicles',type=int,default=4);parser.add_argument('--duration',type=float,default=360)
    parser.add_argument('--no-visualization',action='store_true');parser.add_argument('--sensor-debug',action='store_true')
    parser.add_argument('--network-debug',action='store_true');parser.add_argument('--snapshot')
    parser.add_argument('--scenario',choices=['demo','intersection','dense-fog','v2v-warning','sensor-disagreement','fleet-arrival'],default='demo')
    parser.add_argument('--results',default='results/latest.json');parser.add_argument('--config',help='JSON overrides for Config fields')
    parser.add_argument('--hold',action='store_true',help='Keep GUI open after final parking')
    args=parser.parse_args()
    if not 2<=args.vehicles<=12:parser.error('--vehicles must be between 2 and 12')
    return args

def main():
    args=arguments();cfg=Config(seed=args.seed,vehicles=args.vehicles,headless=args.headless,duration=args.duration,
            visualization=not args.no_visualization,sensor_debug=args.sensor_debug,network_debug=args.network_debug,scenario=args.scenario)
    if args.config:
        data=json.loads(Path(args.config).read_text());valid={f.name for f in fields(Config)}
        if set(data)-valid:raise ValueError(f'Unknown configuration keys: {set(data)-valid}')
        for key,value in data.items():setattr(cfg,key,value)
    if cfg.scenario=='dense-fog':cfg.camera_range=18;cfg.lidar_range=25
    elif cfg.scenario=='v2v-warning':cfg.camera_range=10;cfg.packet_loss=.02
    elif cfg.scenario=='sensor-disagreement':cfg.camera_bias=7.;cfg.packet_loss=.28;cfg.latency_max=.7
    elif cfg.scenario=='intersection':cfg.cruise=5.8
    sim=Simulation(cfg)
    try:
        result=sim.run(args.snapshot)
        path=Path(args.results);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2))
        print('\nFIRST LIGHT RESULTS\n'+json.dumps(result,indent=2))
        if args.hold and sim.hud:
            while p.isConnected():
                if sim.hud.keys(sim.vehicles):break
                sim.hud.update(sim.centre,sim.vehicles,sim.v2v,sim.metrics,sim.t);time.sleep(.1)
        return 0 if result['vehicles_parked']==cfg.vehicles and result['collisions']==0 else 2
    finally:sim.close()

if __name__=='__main__':raise SystemExit(main())
