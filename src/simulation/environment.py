"""Procedural low-poly mine. Geometry is generated once; no downloaded assets."""
import math
import numpy as np

class Environment:
    def __init__(self,p,network,cfg):
        self.p=p;self.network=network;self.cfg=cfg;self.targets=[];self.fog_bodies=[]
        self.rng=np.random.default_rng(cfg.seed+100)
        self.build()
    def box(self,pos,size,color,yaw=0,collision=True):
        p=self.p
        vis=p.createVisualShape(p.GEOM_BOX,halfExtents=size,rgbaColor=color)
        col=p.createCollisionShape(p.GEOM_BOX,halfExtents=size) if collision else -1
        return p.createMultiBody(0,col,vis,pos,p.getQuaternionFromEuler([0,0,yaw]))
    def cylinder(self,pos,radius,height,color):
        p=self.p
        vis=p.createVisualShape(p.GEOM_CYLINDER,radius=radius,length=height,rgbaColor=color)
        col=p.createCollisionShape(p.GEOM_CYLINDER,radius=radius,height=height)
        return p.createMultiBody(0,col,vis,pos)
    def label(self,text,pos,color=(.9,.95,1),size=1.5):
        if not self.cfg.headless:self.p.addUserDebugText(text,pos,color,size)
    def mesh(self,vertices,indices,color):
        p=self.p
        v=p.createVisualShape(p.GEOM_MESH,vertices=vertices,indices=indices,rgbaColor=color)
        c=p.createCollisionShape(p.GEOM_MESH,vertices=vertices,indices=indices,flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        return p.createMultiBody(0,c,v)
    def build(self):
        p=self.p
        self.box([0,0,-12],[205,115,5],[.27,.19,.14,1])
        # Terraced survey mesh, locally cut and filled around every road. This
        # prevents decorative terrain from burying trucks on the lower benches.
        xs=np.arange(-169,202,2.5);ys=np.arange(-109,112,2.5)
        xx,yy=np.meshgrid(xs,ys);xy=np.c_[xx.ravel(),yy.ravel()]
        angle=np.arctan2((xy[:,1]+12)/.62,xy[:,0]+33)
        radius=np.sqrt((xy[:,0]+33)**2+((xy[:,1]+12)/.62)**2)/(1+.025*np.sin(angle*7))
        height=np.select([radius<38,radius<66,radius<94,radius<120],[-1,3,8,13],default=17).astype(float)
        best_distance=np.full(len(xy),np.inf);road_height=np.zeros(len(xy));road_width=np.zeros(len(xy))
        for edge in self.network.edges:
            delta=xy[:,None,:]-edge.points[None,:,:2]
            distance=np.linalg.norm(delta,axis=2);nearest=np.argmin(distance,axis=1)
            d=distance[np.arange(len(xy)),nearest]
            better=d<best_distance
            best_distance[better]=d[better];road_height[better]=edge.points[nearest[better],2];road_width[better]=edge.width
        blend=np.clip((road_width/2+9-best_distance)/6,0,1)
        height=height*(1-blend)+(road_height-.8)*blend
        vertices=np.c_[xy,height].tolist();groups=[[] for _ in range(5)];nx=len(xs)
        for row in range(len(ys)-1):
            for col in range(nx-1):
                j=row*nx+col;band=int(np.searchsorted([38,66,94,120],radius[j]))
                groups[band].extend([j,j+1,j+nx,j+1,j+nx+1,j+nx])
        for indices,color in zip(groups,[[.38,.24,.16,1],[.49,.31,.19,1],[.57,.38,.25,1],[.48,.32,.21,1],[.6,.42,.28,1]]):
            self.mesh(vertices,indices,color)
        # Surveyed haul roads are physical ribbons, cut/fill supported. Remove terrace
        # collision under road corridors via collision masks; roads are actual surfaces.
        terrain_ids=list(range(p.getNumBodies()))
        for bid in terrain_ids:p.setCollisionFilterGroupMask(bid,-1,2,2)
        for edge in self.network.edges:
            pts=edge.points;verts=[];indices=[]
            for i,pt in enumerate(pts):
                delta=pts[min(i+1,len(pts)-1)]-pts[max(i-1,0)]
                n=np.array([-delta[1],delta[0],0]);n/=max(np.linalg.norm(n),1e-8)
                verts.extend([(pt+n*edge.width/2+[0,0,.02]).tolist(),(pt-n*edge.width/2+[0,0,.02]).tolist()])
                if i:
                    j=2*(i-1);indices.extend([j,j+1,j+2,j+1,j+3,j+2])
                if i%9==0:
                    for side in (() if edge.start=='GATE' else (-1,1)):
                        loc=pt+n*(edge.width/2+.5)*side
                        self.box((loc+[0,0,.4]).tolist(),[.6,.45,.4],[.65,.42,.19,1],collision=False)
                    self.box((pt+[0,0,.06]).tolist(),[1.3,.09,.025],[.88,.73,.4,1],math.atan2(delta[1],delta[0]),False)
            self.mesh(verts,indices,[.24,.245,.24,1])
        # Apron and facilities are positioned around the high bench.
        apron_right=max(152,float(self.network.slots[-1][0])+8)
        self.box([(72+apron_right)/2,39,17.7],[(apron_right-72)/2,21,.3],[.30,.32,.32,1])
        for i,slot in enumerate(self.network.slots):
            for dx in (-4,4):self.box((slot+[dx,-3,.06]).tolist(),[.08,7,.035],[.94,.85,.55,1],collision=False)
            self.box((slot+[0,4,.3]).tolist(),[3,.35,.3],[.84,.68,.16,1])
            self.label(f'{i+1:02d}',(slot+[0,5,1]).tolist(),size=1.2)
        self.building('FIRST LIGHT | COMMAND', [112,73,18],[23,8,4],[.15,.28,.33,1],True)
        self.building('MAINTENANCE',[-92,68,13],[15,7,4],[.38,.41,.4,1])
        self.building('ORE PROCESSING',[-108,-73,13],[18,9,6],[.39,.31,.24,1])
        self.building('WAREHOUSE',[57,-48,13],[14,8,5],[.4,.43,.39,1])
        self.building('WORKSHOP',[94,-40,18],[11,7,4],[.38,.4,.37,1])
        for x in (-130,-119,-108):self.cylinder([x,-87,19],3.5,12,[.48,.49,.45,1])
        for x in (55,65):self.cylinder([x,-68,19],3.8,12,[.58,.59,.55,1])
        self.box([-88,-72,24],[28,1.6,1],[.22,.25,.25,1],.23)
        for x in (-112,-96,-80,-64):self.box([x,-72,18],[.35,.5,6],[.33,.34,.31,1])
        self.box([72,69,24],[1,1,6],[.3,.32,.3,1]);self.box([72,69,31],[3,3,1.8],[.23,.4,.43,1])
        # Rocks avoid road corridors. Catalogue is used ONLY by sensor simulators.
        for _ in range(85):
            x,y=self.rng.uniform(-145,65),self.rng.uniform(-85,80)
            if any(np.min(np.linalg.norm(e.points[:,:2]-[x,y],axis=1))<e.width/2+3 for e in self.network.edges):continue
            r=math.sqrt((x+33)**2+((y+12)/.62)**2)
            z=-1 if r<38 else 3 if r<66 else 8 if r<94 else 13 if r<120 else 17
            radius=self.rng.uniform(.7,2.0)
            bid=self.box([x,y,z+radius*.5],[radius,radius*.7,radius*.55],[.28,.25,.23,1],self.rng.uniform(0,6))
            self.targets.append((f'rock-{bid}',bid,np.array([x,y,z+radius*.5]),np.zeros(3),radius))
        # Shoulder work zone: perceived hazard, no artificial blockage of the only route.
        for x in (4,10,16):
            bid=self.box([x,7,14.5],[1.8,.6,1],[.92,.44,.08,1])
            self.targets.append((f'barrier-{bid}',bid,np.array([x,7,14.5]),np.zeros(3),1.8))
        self.label('LOWER PIT / 00 m',[-65,-57,3],[.9,.74,.48],1.6)
        self.label('BENCH 02 / 09 m',[-91,34,14],[.9,.74,.48],1.6)
        self.label('UPPER HAUL / 18 m',[5,77,23],[.9,.74,.48],1.6)
        self.label('JUNCTION J',[-27,-8,12],[1,.74,.22],1.2)
        self.label('JUNCTION K',[35,-8,21],[1,.74,.22],1.2)
        if self.cfg.visualization:
            for bank in range(3):
                for j in range(9):
                    shape=p.createVisualShape(p.GEOM_SPHERE,radius=7+j%3,rgbaColor=[.76,.82,.84,.055])
                    bid=p.createMultiBody(0,-1,shape,[0,0,-100])
                    self.fog_bodies.append((bid,bank,j))
    def building(self,name,base,size,color,command=False):
        x,y,z=base;w,d,h=size
        bid=self.box([x,y,z+h],[w,d,h],color)
        # Facade reflectors avoid treating a long building as a giant spherical
        # obstacle extending across the adjacent parking apron.
        for j,dx in enumerate(np.arange(-w+3,w,6)):
            self.targets.append((f'building-{bid}-{j}',bid,np.array([x+dx,y-d,z+1.65]),np.zeros(3),2.5))
        self.box([x,y,z+2*h+.35],[w+.6,d+.6,.35],[.19,.22,.23,1])
        for dx in np.arange(-w+2,w-1,4):
            self.box([x+dx,y-d-.05,z+h+1],[1.2,.1,1],[.26,.68,.76,1],collision=False)
        if command:
            self.box([x,y-d-.3,z+2*h-1],[w,.25,.35],[.1,.8,.79,1],collision=False)
            self.cylinder([x+w-2,y,z+2*h+4],.15,8,[.7,.76,.77,1])
        self.label(name,[x-w,y-d-.5,z+2*h+1],(.3,.95,.92) if command else (.85,.87,.8),1.6)
    def update_fog(self,fog,t):
        for bid,bank,j in self.fog_bodies:
            angle=j*2.4;center=fog.centres[bank]+[math.cos(angle)*11,math.sin(angle)*9,3+j%3*2]
            self.p.resetBasePositionAndOrientation(bid,center.tolist(),[0,0,0,1])
            # Visual scale is pre-built via radius; alpha conveys density, not collision.
            self.p.changeVisualShape(bid,-1,rgbaColor=[.77,.83,.85,.055+fog.density[bank]*.035])
