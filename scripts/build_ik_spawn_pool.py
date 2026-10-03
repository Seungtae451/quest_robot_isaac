"""Build exact XY spawn locations with the live teleop IK settings and camera.

HOME -> hover -> grasp -> lift -> above box -> release are checked at Cartesian
waypoints. This is kinematic feasibility, not collision planning/grasp success.
The Isaac process only reads this file; it never imports Pinocchio.
"""
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from config import tabletop_config as table
from config import teleop_config as cfg
from robot.f14_config import F14_URDF_PATH, HOME_Q, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.f14_ik import F14IK
from robot.spawn_workspace import CACHE_PATH, workspace_signature, grasp_wrist_position, cube_is_in_camera
from simulation.tabletop import cube_spawn_regions, table_surface_height


def verify_route(ik, x, y, side):
    surface=table_surface_height();cube_height=surface+table.CUBE_SIZE/2
    grasp=grasp_wrist_position(x,y,side,cube_height)
    hover=grasp+[0,0,table.GRASP_APPROACH_HEIGHT]
    lift=grasp+[0,0,table.GRASP_LIFT_HEIGHT]
    bx,by=table.BOX_CENTER_XY
    release_height=surface+table.BOX_SIZE[2]+table.CUBE_SIZE/2+table.BOX_RELEASE_GAP
    release=grasp_wrist_position(bx,by,side,release_height)
    above=release.copy();above[2]=max(release[2],lift[2])
    waypoints=[hover,grasp,lift,above,release]
    home=ik.forward_kinematics(HOME_Q)
    current=home[side].translation.copy();q=HOME_Q.copy();endpoints=[]
    lower=ik.model.lowerPositionLimit[ik.arm_q_indices]
    upper=ik.model.upperPositionLimit[ik.arm_q_indices]
    maximum_step=0.
    for goal in waypoints:
        start=current.copy()
        count=max(1,math.ceil(np.linalg.norm(goal-start)/table.IK_SPAWN_PATH_STEP))
        for t in np.linspace(0,1,count+1)[1:]:
            targets=[p.copy() for p in home]
            for i,target in enumerate(targets):
                target.rotation=(LEFT_EE_DOWN_ROT,RIGHT_EE_DOWN_ROT)[i].copy()
            targets[side].translation=start+t*(goal-start)
            solution,ok=ik.solve(*targets,q,max_iter=cfg.IK_MAX_ITER,eps=cfg.IK_EPS,
                                dt=cfg.IK_DT,damping=cfg.IK_DAMPING)
            if not ok:
                return None,"ik"
            step=float(np.max(np.abs(solution-q)))
            if step>table.IK_SPAWN_MAX_JOINT_STEP:
                return None,"branch_step"
            if np.min(np.minimum(solution-lower,upper-solution))<table.IK_SPAWN_JOINT_MARGIN:
                return None,"joint_margin"
            maximum_step=max(maximum_step,step);q=solution
        current=goal.copy();endpoints.append(q.tolist())
    return {"x":x,"y":y,"side":("left","right")[side],
            "route_q":endpoints,"max_joint_step_rad":maximum_step},None


def main():
    started=time.monotonic();ik=F14IK(F14_URDF_PATH)
    candidates=[];counts={"geometric":0,"camera":0,"ik":0,"branch_step":0,"joint_margin":0}
    radius=table.CUBE_SIZE/np.sqrt(2);step=table.IK_SPAWN_GRID_STEP
    for xmin,xmax,ymin,ymax in cube_spawn_regions():
        xs=np.arange(math.ceil((xmin+radius)/step)*step,xmax-radius+1e-9,step)
        ys=np.arange(math.ceil((ymin+radius)/step)*step,ymax-radius+1e-9,step)
        for x in xs:
            for y in ys:
                x,y=round(float(x),9),round(float(y),9)
                if abs(y-table.BOX_CENTER_XY[1])<table.BOX_SIZE[1]/2+table.CUBE_GRIPPER_BOX_CLEARANCE:
                    continue
                counts["geometric"]+=1
                if not cube_is_in_camera(x,y,table_surface_height()):
                    counts["camera"]+=1;continue
                side=0 if y>table.BOX_CENTER_XY[1] else 1
                candidate,reason=verify_route(ik,x,y,side)
                if candidate is None:
                    counts[reason]+=1
                else:
                    candidates.append(candidate)
        print(f"IK spawn check: {len(candidates)} accepted, {counts['geometric']} tested",flush=True)
    if not candidates or {p["side"] for p in candidates}!={"left","right"}:
        raise RuntimeError(f"No usable IK spawn locations for both box sides: {counts}")
    summary={name:{"count":len([p for p in candidates if p['side']==name]),
        "xy_bounds_m":np.array([[p['x'],p['y']] for p in candidates if p['side']==name]).min(0).tolist()+
                       np.array([[p['x'],p['y']] for p in candidates if p['side']==name]).max(0).tolist()}
             for name in ("left","right")}
    data={"signature":workspace_signature(),"candidates":candidates,"summary":summary,"rejections":counts,
          "elapsed_seconds":time.monotonic()-started,"route":["hover","grasp","lift","above_box","release"],
          "collision_checked":False,"camera_occlusion_checked":False}
    CACHE_PATH.parent.mkdir(parents=True,exist_ok=True)
    temp=CACHE_PATH.with_suffix(f".{os.getpid()}.tmp")
    temp.write_text(json.dumps(data,separators=(",",":"))+"\n")
    temp.replace(CACHE_PATH)
    print(json.dumps({"pool":str(CACHE_PATH),"summary":summary,"rejections":counts,
                      "elapsed_seconds":data['elapsed_seconds']},indent=2),flush=True)
    plot_pool(data)


def plot_pool(data):
    os.environ.setdefault("MPLCONFIGDIR","/tmp/f14-spawn-matplotlib")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from robot.spawn_workspace import camera_pixels
    fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
    ax=axes[0];cx,cy,_=table.TABLE_CENTER;sx,sy,_=table.TABLE_SIZE
    ax.add_patch(Rectangle((cx-sx/2,cy-sy/2),sx,sy,fill=False,color='#91623c',linewidth=2,label='New table'))
    bx,by=table.BOX_CENTER_XY;bsx,bsy,_=table.BOX_SIZE
    ax.add_patch(Rectangle((bx-bsx/2,by-bsy/2),bsx,bsy,facecolor='#8ccc8c',alpha=.3,label='Collection box'))
    for xmin,xmax,ymin,ymax in cube_spawn_regions():
        ax.add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,fill=False,linestyle='--',edgecolor='#aaa'))
    for name,color in [('left','#e69f00'),('right','#0072b2')]:
        points=np.array([[p['x'],p['y']] for p in data['candidates'] if p['side']==name])
        ax.scatter(*points.T,s=16,color=color,label=f'{name}: {len(points)} verified spawn locations')
        uv,_=camera_pixels(np.column_stack((points,np.full(len(points),table_surface_height()+table.CUBE_SIZE/2))))
        axes[1].scatter(*uv.T,s=16,color=color,label=name)
    ax.set(xlabel='X forward (m)',ylabel='Y left (m)',title='Exact cube centers: approach / grasp / lift / place IK')
    ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid(alpha=.2)
    ax=axes[1];margin=table.SPAWN_CAMERA_MARGIN
    ax.add_patch(Rectangle((cfg.BODY_CAMERA_WIDTH*margin,cfg.BODY_CAMERA_HEIGHT*margin),
        cfg.BODY_CAMERA_WIDTH*(1-2*margin),cfg.BODY_CAMERA_HEIGHT*(1-2*margin),fill=False,color='#777',linestyle='--',label='Camera margin'))
    ax.set(xlim=(0,cfg.BODY_CAMERA_WIDTH),ylim=(cfg.BODY_CAMERA_HEIGHT,0),
           xlabel='Camera pixel X',ylabel='Camera pixel Y',title='Ideal body camera projection (occlusion not checked)')
    ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.savefig(CACHE_PATH.parent/'ik_spawn_coverage.png',dpi=160)
    plt.close(fig)


if __name__=="__main__":
    main()
