"""Isaac-only collider/velocity observations for collection assistance.

No drives, efforts, joint states or dataset fields are written here. The Quest
sender limits its XYZ target before its existing fixed-orientation IK solve.
"""
import itertools
import time

import numpy as np

from config import table_assist_config as cfg, tabletop_config as table
from robot.f14_config import EE_BODY_NAMES, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from simulation.tabletop import table_surface_height, box_parts


def rotation(q):
    w,x,y,z=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


class TableApproachMonitor:
    def __init__(self,robot):
        import omni.usd
        from pxr import Usd,UsdGeom,UsdPhysics
        from simulation.cameras import rigid_link_path
        self.robot=robot
        self.ee_ids,names=robot.find_bodies(EE_BODY_NAMES,preserve_order=True)
        self.parts=[]
        self.obstacles=[{'name':'box_'+name,'bounds':[(np.asarray(pos)-np.asarray(size)/2).tolist(),
                                                      (np.asarray(pos)+np.asarray(size)/2).tolist()]}
                        for name,size,pos in box_parts()]
        self.obstacles.append({'name':'table','bounds':[
            (np.asarray(table.TABLE_CENTER)-np.asarray(table.TABLE_SIZE)/2).tolist(),
            (np.asarray(table.TABLE_CENTER)+np.asarray(table.TABLE_SIZE)/2).tolist()]})
        stage=omni.usd.get_context().get_stage()
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide],
                               False,True)  # include invisible collision meshes (USD 24.08 positional API)
        for side in ('left','right'):
            parts=[]
            for name in (f'{side}_dof7_link',f'{side}_l_gripper_link',f'{side}_r_gripper_link'):
                ids,resolved=robot.find_bodies([name],preserve_order=True)
                if resolved!=[name]:
                    raise ValueError(f'Collection assist body lookup failed: {name}')
                body=stage.GetPrimAtPath(rigid_link_path(name))
                corners=[]
                for prim in Usd.PrimRange(body,Usd.TraverseInstanceProxies()):
                    if prim.HasAPI(UsdPhysics.CollisionAPI):
                        bounds=cache.ComputeRelativeBound(prim,body).ComputeAlignedRange()
                        if not bounds.IsEmpty():
                            low,high=np.asarray(bounds.GetMin()),np.asarray(bounds.GetMax())
                            corners.extend(itertools.product(*zip(low,high)))
                # The canonical USD has a wrist actor but may omit its collider.
                # Finger colliders remain mandatory; an absent wrist collider
                # cannot contact the table and need not enter the envelope.
                if not corners and name.endswith('dof7_link'):
                    continue
                if not corners or not np.isfinite(corners).all():
                    schema_info=[(str(p.GetPath()),p.GetTypeName(),p.GetAppliedSchemas())
                                 for p in Usd.PrimRange(body,Usd.TraverseInstanceProxies())]
                    raise ValueError(f'No finite collision envelope for {name}: {schema_info}')
                parts.append((ids[0],np.asarray(corners,float)))
            self.parts.append(parts)
        self.previous=None
        self.filtered=None
        self.status='TABLE/BOX ASSIST OFF'

    def sample(self,dt,enabled):
        timestamp=time.monotonic()
        if not enabled:
            self.previous=self.filtered=None
            self.status='TABLE/BOX ASSIST OFF'
            return {'enabled':False,'timestamp':timestamp}
        data=self.robot.data
        positions=data.body_link_pos_w[0].cpu().numpy()
        quaternions=data.body_link_quat_w[0].cpu().numpy()
        # Actor-origin velocities avoid confusing link-7 COM velocity with its frame origin.
        velocities=data.body_link_vel_w[0,self.ee_ids].cpu().numpy()
        points=[];part_points=[]
        for parts in self.parts:
            transformed=[corners@rotation(quaternions[body_id]).T+positions[body_id]
                         for body_id,corners in parts]
            part_points.append(transformed)
            points.append(np.concatenate(transformed))
        # PhysX solver impulses may report velocity on a stationary constraint.
        # Use the actual observed collider displacement for the predictive aid;
        # also expose the raw Isaac link-7 linear/angular velocity for diagnosis.
        estimated=[np.zeros_like(p) for p in points] if self.previous is None else [
            (p-previous)/dt for p,previous in zip(points,self.previous)]
        alpha=1-np.exp(-dt/cfg.VELOCITY_FILTER_SECONDS)
        self.filtered=estimated if self.filtered is None else [
            old+alpha*(new-old) for old,new in zip(self.filtered,estimated)]
        self.previous=[p.copy() for p in points]
        x,y,_=table.TABLE_CENTER
        length,width,_=table.TABLE_SIZE
        arms=[]
        for index,(point,velocity,ee_id,down) in enumerate(zip(points,self.filtered,self.ee_ids,
                                                             (LEFT_EE_DOWN_ROT,RIGHT_EE_DOWN_ROT))):
            wrist=positions[ee_id]
            local=(point-wrist)@rotation(quaternions[ee_id])
            offsets=local@down.T
            part_offsets=[(p-wrist)@rotation(quaternions[ee_id])@down.T for p in part_points[index]]
            closing=max(0.,float(-np.min(velocity[:,2])))
            box_gap=min(float(np.linalg.norm(np.maximum(np.maximum(
                np.asarray(obstacle['bounds'])[0]-p.max(axis=0),
                p.min(axis=0)-np.asarray(obstacle['bounds'])[1]),0.)))
                for p in part_points[index] for obstacle in self.obstacles if obstacle['name'].startswith('box_'))
            over_table=(point[:,0].max()>=x-length/2 and point[:,0].min()<=x+length/2
                        and point[:,1].max()>=y-width/2 and point[:,1].min()<=y+width/2)
            arms.append({'wrist_position':wrist.tolist(),'wrist_velocity':velocities[index].tolist(),
                         'offset_bounds':[offsets.min(axis=0).tolist(),offsets.max(axis=0).tolist()],
                         'part_bounds':[[p.min(axis=0).tolist(),p.max(axis=0).tolist()] for p in part_offsets],
                         'velocity_bounds':[velocity.min(axis=0).tolist(),velocity.max(axis=0).tolist()],
                         'box_clearance':box_gap,
                         'closing_speed':closing,'clearance':float(point[:,2].min()-table_surface_height()),
                         'predicted_clearance':float((point[:,2]+velocity[:,2]*cfg.LOOKAHEAD_SECONDS).min()-table_surface_height()),
                         'over_table':bool(over_table)})
        payload={'enabled':True,'timestamp':timestamp,'surface_z':table_surface_height(),
                 'table_xy':[x-length/2,x+length/2,y-width/2,y+width/2],
                 'obstacles':self.obstacles,'arms':arms}
        labels=[]
        for side,arm in zip(('L','R'),arms):
            if not arm['over_table']:
                labels.append(f'{side} OUTSIDE')
                continue
            risk=arm['predicted_clearance']<cfg.MINIMUM_CLEARANCE
            labels.append(f'{side} T{arm["clearance"]*1000:.1f}/B{arm["box_clearance"]*1000:.1f}mm'+(' BRAKE' if risk else ''))
        self.status='TABLE/BOX ASSIST '+' / '.join(labels)
        return payload
