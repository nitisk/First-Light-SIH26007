"""Depth-aware fog post-process for TinyRenderer exports (which ignores alpha).

The interactive OpenGL GUI uses transparent world-space wisps. Exported frames
integrate the SAME Gaussian fog field along each camera ray, not a painted mask.
"""
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont

def export_frame(sim,path,parking=False):
    import pybullet as p
    width,height=1600,900
    target,distance,yaw,pitch=([108,49,19],86,12,-40) if parking else ([5,6,10],235,43,-49)
    view=p.computeViewMatrixFromYawPitchRoll(target,distance,yaw,pitch,0,2)
    projection=p.computeProjectionMatrixFOV(53,width/height,.5,600)
    # TinyRenderer renders translucent spheres as solid white balls. Hide only
    # those visual objects and replace them with analytical extinction below.
    for bid,_,_ in sim.environment.fog_bodies:p.resetBasePositionAndOrientation(bid,[0,0,-200],[0,0,0,1])
    _,_,rgba,depth,_=p.getCameraImage(width,height,view,projection,renderer=p.ER_TINY_RENDERER,shadow=1,
             lightDirection=[-1,-1,3],lightColor=[1,.97,.91],lightAmbientCoeff=.65,lightDiffuseCoeff=.65)
    rgb=np.asarray(rgba,dtype=float)[:,:,:3]
    rgb[np.asarray(depth)>.9999]=[159,185,196]
    if sim.cfg.visualization:
        v=np.asarray(view).reshape(4,4,order='F');proj=np.asarray(projection).reshape(4,4,order='F')
        x,y=np.meshgrid(np.linspace(-1,1,width),np.linspace(1,-1,height))
        clip=np.stack([x,y,2*np.asarray(depth)-1,np.ones_like(x)],axis=-1)
        world=clip@np.linalg.inv(proj@v).T;world=world[:,:,:3]/world[:,:,3:]
        eye=np.linalg.inv(v)[:3,3];ray=world-eye
        length=np.linalg.norm(ray,axis=-1);extinction=np.zeros((height,width))
        for f in np.linspace(.06,.98,12):
            point=eye+ray*f
            for centre,density in zip(sim.fog.centres,sim.fog.density):
                delta=(point-(centre+[0,0,3]))/[24,21,6]
                extinction+=density*np.exp(-.5*np.sum(delta*delta,axis=-1))*length/12*.022
        alpha=1-np.exp(-extinction)
        rgb=rgb*(1-alpha[:,:,None])+np.array([191,207,214])*alpha[:,:,None]
    im=Image.fromarray(np.uint8(np.clip(rgb,0,255))).convert('RGB');draw=ImageDraw.Draw(im)
    def font(size):
        for name in ('C:/Windows/Fonts/segoeui.ttf','DejaVuSans.ttf'):
            try:return ImageFont.truetype(name,size)
            except OSError:pass
        return ImageFont.load_default()
    draw.rectangle([0,0,width,104],fill='#101e26')
    draw.text((35,13),'FIRST LIGHT',font=font(34),fill='#69e5d2')
    draw.text((36,59),'PREDICTIVE FOG-ADAPTIVE COOPERATIVE MINE SAFETY',font=font(17),fill='#b6cbd3')
    draw.text((1080,25),f'T + {sim.t:06.1f} s   |   '+('FLEET ARRIVAL' if parking else 'MINE OPERATIONS'),font=font(21),fill='#e8f1f2')
    draw.rectangle([0,height-71,width,height],fill='#101e26')
    parked=sum(v.parked for v in sim.vehicles)
    draw.text((35,height-54),f'{parked}/{len(sim.vehicles)} PARKED     {len(sim.metrics.collisions)} COLLISIONS     {len(sim.metrics.avoided)} RESOLVED CONFLICTS',font=font(22),fill='#69e5d2')
    draw.text((895,height-51),f'V2V {sim.v2v.received:,} RX   |   CAMERA + LiDAR + RADAR',font=font(19),fill='#d4e2e7')
    Path(path).parent.mkdir(parents=True,exist_ok=True);im.save(path)
    if not sim.cfg.headless:sim.environment.update_fog(sim.fog,sim.t)
