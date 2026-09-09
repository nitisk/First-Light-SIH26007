"""Generic force-driven haul truck. Truth is confined to dynamics and sensor sampling."""
import math
import numpy as np
from perception import Perception
from positioning import Positioning
from risk_map import RiskMap
from response import Response
from fusion import fuse
from prediction import predict,Decision

COLORS=[(.96,.63,.10,1),(.93,.43,.13,1),(.76,.79,.19,1),(.13,.64,.68,1),(.58,.42,.72,1),(.72,.66,.51,1)]

class Vehicle:
    def __init__(self,p,i,route,cfg):
        self.p=p;self.id=f'V{i+1}';self.index=i;self.route=route;self.cfg=cfg
        self.progress=0.;self.position=route.sample(0)+[0,0,1.65];self.velocity=np.zeros(3)
        direction=route.direction(0);self.yaw=math.atan2(direction[1],direction[0]);self.speed=0
        self.body=self.build(COLORS[i%len(COLORS)])
        self.perception=Perception(cfg,np.random.default_rng(cfg.seed+17*i))
        self.localization=Positioning();self.risk_map=RiskMap(self.id,cfg.map_ttl)
        self.inbox={};self.response=Response();self.decision=Decision();self.measurements=[];self.tracks=[]
        self.visibility=70.;self.target_speed=0.;self.parked=False;self.parking_state='EN ROUTE'
        self.last_sensor=-1e9;self.fog_warning=False;self.fog_warning_events=0
        self.estimated_progress=0.;self.measured_heading=self.yaw
        self.last_command_ack=-1e9;self.command_connection='OFFLINE'
        self.release_time=(i//4)*18.0
        # Extra trucks occupy separate longitudinal positions on their approach.
        if i>=4:
            self.release_time=0.;self.progress=(i//4)*18.;self.position=route.sample(self.progress)+[0,0,1.65]
            direction=route.direction(self.progress);self.yaw=math.atan2(direction[1],direction[0])
            p.resetBasePositionAndOrientation(self.body,self.position.tolist(),p.getQuaternionFromEuler([0,0,self.yaw]))
    def build(self,color):
        p=self.p
        types=[];half=[];positions=[];orientations=[];colors=[];radii=[];lengths=[]
        def part(pos,size,c,kind=None,r=.1,length=.1,rotation=(0,0,0)):
            types.append(p.GEOM_BOX if kind is None else kind);half.append(size);positions.append(pos)
            orientations.append(p.getQuaternionFromEuler(rotation));colors.append(c);radii.append(r);lengths.append(length)
        part([0,0,0],[3.4,1.65,.45],[.15,.17,.16,1])
        part([-.8,0,.95],[2.5,1.9,.38],color)
        part([-.8,-1.8,1.55],[2.55,.15,.65],color);part([-.8,1.8,1.55],[2.55,.15,.65],color)
        part([-3.2,0,1.55],[.17,1.8,.65],color);part([1.5,0,1.55],[.16,1.8,.65],color)
        part([2.2,.5,1.25],[.9,1,.95],color);part([3.12,.5,1.55],[.03,.84,.47],[.1,.26,.31,1])
        part([2.2,1.52,1.55],[.66,.03,.47],[.13,.33,.37,1])
        part([2.2,.5,2.25],[1.03,1.15,.12],color)
        for x in (-2.1,2.0):
            for y in (-1.85,1.85):
                part([x,y,-.55],[0,0,0],[.065,.072,.075,1],p.GEOM_CYLINDER,1.05,.7,(math.pi/2,0,0))
                part([x,y*1.19,-.55],[0,0,0],[.5,.51,.47,1],p.GEOM_CYLINDER,.47,.05,(math.pi/2,0,0))
        for y in (-1.1,1.1):part([3.45,y,.05],[.06,.28,.15],[1,.93,.57,1])
        part([2.2,.5,2.51],[.16,.16,.16],[1,.48,.07,1])
        part([1.8,-1.1,2.1],[.15,.15,.28],[.25,.3,.31,1])
        # Bullet compound visual arrays have a small implementation limit. Fixed
        # visual-only links keep each batch below it and preserve one rigid chassis.
        visuals=[]
        for begin in range(0,len(types),12):
            sl=slice(begin,begin+12)
            visuals.append(p.createVisualShapeArray(shapeTypes=types[sl],halfExtents=half[sl],visualFramePositions=positions[sl],
                    visualFrameOrientations=orientations[sl],rgbaColors=colors[sl],radii=radii[sl],lengths=lengths[sl]))
        collision=p.createCollisionShape(p.GEOM_BOX,halfExtents=[3.4,1.9,.8],collisionFramePosition=[0,0,.25])
        count=len(visuals)-1
        body=p.createMultiBody(8000,collision,visuals[0],self.position.tolist(),p.getQuaternionFromEuler([0,0,self.yaw]),
                linkMasses=[0]*count,linkCollisionShapeIndices=[-1]*count,linkVisualShapeIndices=visuals[1:],
                linkPositions=[[0,0,0]]*count,linkOrientations=[[0,0,0,1]]*count,
                linkInertialFramePositions=[[0,0,0]]*count,linkInertialFrameOrientations=[[0,0,0,1]]*count,
                linkParentIndices=[0]*count,linkJointTypes=[p.JOINT_FIXED]*count,linkJointAxis=[[0,0,0]]*count)
        p.changeDynamics(body,-1,linearDamping=0,angularDamping=.8,lateralFriction=.7,restitution=0)
        return body
    def truth(self):
        pos,q=self.p.getBasePositionAndOrientation(self.body);vel,ang=self.p.getBaseVelocity(self.body)
        self.position=np.array(pos);self.velocity=np.array(vel);euler=self.p.getEulerFromQuaternion(q)
        self.yaw=euler[2];self.speed=float(np.linalg.norm(self.velocity[:2]))
        return euler,ang
    def receive(self,packet,t):
        old=self.inbox.get(packet['vehicle_id'])
        if old is None or old['timestamp']<packet['timestamp']:
            self.inbox[packet['vehicle_id']]=packet;self.risk_map.merge(packet['reports'],t)
    def sense_decide(self,targets,fog,t):
        self.command_connection='ONLINE' if t-self.last_command_ack<2 else 'STALE'
        euler,ang=self.truth()
        imu=self.perception.imu.update(self.velocity,euler,ang,t)
        gnss=self.perception.gnss.update(self.position,t)
        speed=self.perception.speed.update(self.speed,t)
        self.localization.update(gnss,speed,imu,t)
        self.measured_heading=imu['orientation'][2]
        self.estimated_progress=self.route.project(self.localization.position-[0,0,1.65],self.estimated_progress)
        self.progress=self.route.project(self.position-[0,0,1.65],self.progress)
        self.visibility=fog.visibility(self.position)
        self.risk_map.expire(t)
        self.inbox={k:v for k,v in self.inbox.items() if t-v['timestamp']<=self.cfg.packet_ttl}
        if t-self.last_sensor>=self.cfg.sensor_period-1e-8:
            self.last_sensor=t
            self.measurements=self.perception.scan(self.p,self.body,self.position,self.yaw,self.localization,targets,self.visibility,t)
            if self.visibility<40:self.risk_map.observe(self.localization.position,'fog',1-self.visibility/70,t)
            for m in self.measurements:
                if not m.target.startswith('V'):self.risk_map.observe(m.position,'obstacle',.7,t)
        self.tracks=fuse(self.measurements,self.inbox,t,self.cfg)
        self.decision=predict(self.id,self.localization.position,self.localization.velocity,self.route,self.estimated_progress,self.tracks,self.cfg)
        warning=self.risk_map.fog_ahead(self.route,self.estimated_progress,t)
        self.fog_warning_events+=int(warning and not self.fog_warning);self.fog_warning=warning
        remaining=self.route.length-self.progress
        self.target_speed=self.response.update(self.decision,self.visibility,warning,remaining,self.speed,self.cfg,self.cfg.dt)
        if t<self.release_time:self.target_speed=0
        if self.route.segment(self.progress).startswith('GATE'):self.parking_state='PARKING';self.target_speed=min(self.target_speed,2.4)
        if remaining<1.3 and np.linalg.norm(self.position[:2]-self.route.points[-1,:2])<1.3 and self.speed<.18:
            self.parked=True;self.parking_state='PARKED'
        if self.parked:self.target_speed=0
        assert np.isfinite(self.position).all() and np.isfinite(self.velocity).all(),f'{self.id}: nonfinite dynamics'
    def actuate(self):
        p=self.p;self.truth()
        lookahead=max(2.5,self.speed*1.1)
        goal=self.route.sample(self.progress+lookahead)
        delta=goal-self.position;desired=math.atan2(delta[1],delta[0])
        error=(desired-self.yaw+math.pi)%(2*math.pi)-math.pi
        desired_speed=self.target_speed*max(.2,math.cos(error))
        heading=np.array([math.cos(self.yaw),math.sin(self.yaw)])
        acceleration=(heading*desired_speed-self.velocity[:2])*3
        norm=np.linalg.norm(acceleration)
        if norm>self.cfg.braking:acceleration*=self.cfg.braking/norm
        surface=self.route.sample(self.progress)
        target_z=surface[2]+1.65
        az=9.81+28*(target_z-self.position[2])-10*self.velocity[2]
        p.applyExternalForce(self.body,-1,[8000*acceleration[0],8000*acceleration[1],8000*az],self.position.tolist(),p.WORLD_FRAME)
        roll,pitch,yaw=p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.body)[1]);angular=p.getBaseVelocity(self.body)[1]
        slope=self.route.direction(self.progress)
        pitch_target=-math.asin(float(np.clip(slope[2],-.4,.4)))
        yaw_rate=float(np.clip(error*1.7,-.38,.38)) if not self.parked else 0
        inertia=p.getDynamicsInfo(self.body,-1)[2]
        torque=[inertia[0]*(-10*roll-6*angular[0]),inertia[1]*(10*(pitch_target-pitch)-6*angular[1]),inertia[2]*5*(yaw_rate-angular[2])]
        p.applyExternalTorque(self.body,-1,torque,p.WORLD_FRAME)
    def packet(self,t):
        junction,distance=self.route.next_junction(self.estimated_progress)
        return dict(vehicle_id=self.id,position=self.localization.position.tolist(),velocity=self.localization.velocity.tolist(),
                    position_variance=self.localization.variance,heading=self.measured_heading,speed=self.perception.speed.value,
                    visibility=self.visibility,route_segment=self.route.segment(self.estimated_progress),risk=self.decision.risk,
                    timestamp=t,trajectory=self.route.future(self.estimated_progress,self.perception.speed.value),
                    junction=junction,junction_distance=distance if math.isfinite(distance) else None,reports=self.risk_map.reports())
    def telemetry(self,t):
        data=self.packet(t)
        data.update(elevation=float(self.localization.position[2]-1.65),route_progress=self.estimated_progress/self.route.length,
                    sensors=dict(self.perception.status,gnss_age=t-self.perception.gnss.timestamp),
                    ttc=self.decision.ttc if math.isfinite(self.decision.ttc) else None,braking=self.response.braking,
                    parking=self.parking_state,destination='COMMAND CENTER',v2v_peers=len(self.inbox),
                    fog_ahead=self.fog_warning,target_speed=self.target_speed,reason=self.decision.reason,
                    threat_distance=self.decision.distance if math.isfinite(self.decision.distance) else None,
                    command_connection=self.command_connection)
        return data
