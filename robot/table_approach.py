"""Limit Cartesian collection commands before IK; never apply external forces."""
import numpy as np

from config import table_assist_config as cfg


def segment_entry(start, end, low, high):
    """First entry of a segment into a closed AABB, including thin walls."""
    delta=end-start
    entry,leave,axis=0.,1.,None
    for i in range(3):
        if abs(delta[i])<1e-12:
            if start[i]<low[i] or start[i]>high[i]:
                return None
            continue
        near,far=sorted(((low[i]-start[i])/delta[i],(high[i]-start[i])/delta[i]))
        if near>=entry:
            entry,axis=near,i
        leave=min(leave,far)
        if entry>leave:
            return None
    return (entry,axis) if 0<=entry<=1 and leave>=0 else None


def limit_obstacle_path(start, end, parts, obstacles, velocity_bounds):
    """Sweep individual gripper envelopes; stop before the earliest solid.

    Minkowski expansion converts finite gripper boxes to a point path. The
    hollow collection box remains five solids, so its open top stays usable.
    No route is invented: the operator can raise the hand and go around a wall.
    """
    delta=end-start
    original_end=end.copy()
    original_delta=delta.copy()
    # An envelope can already overlap a margin at HOME or after tracking lag.
    # Allow escape towards ANY near face (including above the rim), while
    # removing components that move deeper towards a solid's centre. Project
    # against all initial overlaps BEFORE sweeping the resulting whole path.
    for part in parts:
        for obstacle in obstacles:
            bounds=np.asarray(obstacle['bounds'],float)
            if bounds.shape!=(2,3) or not np.isfinite(bounds).all() or np.any(bounds[1]<=bounds[0]):
                raise ValueError('Invalid obstacle bounds')
            low=bounds[0]-part[1]-cfg.MINIMUM_CLEARANCE
            high=bounds[1]-part[0]+cfg.MINIMUM_CLEARANCE
            if np.all(start>=low) and np.all(start<=high):
                centre=(low+high)*.5
                inward=(start-centre)*delta < -1e-12
                delta[inward]=0.
    end=start+delta
    fraction=1.
    for part in parts:
        for obstacle in obstacles:
            bounds=np.asarray(obstacle['bounds'],float)
            if bounds.shape!=(2,3) or not np.isfinite(bounds).all() or np.any(bounds[1]<=bounds[0]):
                raise ValueError('Invalid obstacle bounds')
            low=bounds[0]-part[1]-cfg.MINIMUM_CLEARANCE
            high=bounds[1]-part[0]+cfg.MINIMUM_CLEARANCE
            if np.all(start>=low) and np.all(start<=high):
                # The projected path cannot deepen this initial overlap.
                # Other solids are still checked below, including the floor.
                continue
            hit=segment_entry(start,end,low,high)
            if hit is None:
                continue
            entry,axis=hit
            if axis is None:
                fraction=0.
                continue
            speed=max(0.,velocity_bounds[1,axis] if delta[axis]>0 else -velocity_bounds[0,axis])
            fraction=min(fraction,max(0.,entry-speed*cfg.LOOKAHEAD_SECONDS/abs(delta[axis])-1e-7))
    limited=fraction<1. or not np.array_equal(delta,original_delta)
    return (start+fraction*delta,True) if limited else (original_end,False)


def limit_targets(targets, feedback, boot_time, now, *, tcp_only=False):
    """Return copied pose targets, per-arm limit flags and feedback availability.

    Use the measured envelope about the grasp tip when TCP feedback is enabled.
    Legacy wrist feedback remains available to offline diagnostics.
    """
    result=[target.copy() for target in targets]
    limited=[False,False]
    if not isinstance(feedback,dict):
        return result,limited,False
    try:
        fresh=(feedback.get('boot_time')==boot_time
               and 0<=now-float(feedback.get('timestamp',0))<=cfg.FEEDBACK_TIMEOUT)
    except (TypeError,ValueError):
        fresh=False
    if not fresh:
        return result,limited,False
    if not feedback.get('enabled'):
        return result,limited,True
    try:
        table=np.asarray(feedback['table_xy'],float)
        surface=float(feedback['surface_z'])
        arms=feedback['arms']
        if table.shape!=(4,) or len(arms)!=2 or not np.isfinite([*table,surface]).all():
            raise ValueError('Invalid table geometry')
        for index,(target,arm) in enumerate(zip(result,arms)):
            if tcp_only and ('tcp_position' not in arm or 'part_local_points' not in arm):
                raise ValueError('Restart Isaac for current grasp-tip collision feedback')
            offset=np.asarray(arm['offset_bounds'],float)
            wrist=np.asarray(arm.get('tcp_position',arm['wrist_position']),float)
            closing=float(arm['closing_speed'])
            if offset.shape!=(2,3) or wrist.shape!=(3,) or not np.isfinite([*offset.ravel(),*wrist,closing]).all():
                raise ValueError('Invalid wrist prediction')
            # Envelopes at both start and end: protects approach across a table edge.
            low=np.minimum(wrist[:2],target.translation[:2])+offset[0,:2]
            high=np.maximum(wrist[:2],target.translation[:2])+offset[1,:2]
            overlaps=high[0]>=table[0] and low[0]<=table[1] and high[1]>=table[2] and low[1]<=table[3]
            floor=surface-offset[0,2]+cfg.MINIMUM_CLEARANCE
            # Predictive braking may hold the present height, but cannot push a
            # safe wrist upward merely because it is descending. Inside the
            # clearance margin the corrected command gently requests recovery.
            buffer=min(max(closing,0.)*cfg.LOOKAHEAD_SECONDS,max(wrist[2]-floor,0.))
            minimum=floor+buffer
            if overlaps and target.translation[2]<minimum:
                target.translation[2]=minimum
                limited[index]=True
            obstacles=feedback.get('obstacles',[])
            parts=np.asarray(arm.get('part_bounds',[offset]),float)
            velocity=np.asarray(arm.get('velocity_bounds',np.zeros((2,3))),float)
            if (parts.ndim!=3 or parts.shape[1:]!=(2,3) or not len(parts)
                    or velocity.shape!=(2,3) or not np.isfinite(parts).all()
                    or not np.isfinite(velocity).all() or np.any(parts[:,1]<parts[:,0])):
                raise ValueError('Invalid gripper sweep geometry')
            corrected,blocked=limit_obstacle_path(wrist,target.translation,parts,obstacles,velocity)
            target.translation[:]=corrected
            limited[index] |= blocked
    except (KeyError,TypeError,ValueError,IndexError):
        return [target.copy() for target in targets],[False,False],False
    return result,limited,True
