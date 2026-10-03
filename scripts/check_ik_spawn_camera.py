"""Bounded actual Isaac camera/HOME validation for the new IK spawn layout.

--suite runs four isolated cases (close to each HOME, outer edge, far edge).
Otherwise forwards CLI arguments to the real Isaac app with a read-only capture
callback. Never sends arm targets or records a demonstration.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def suite():
    output=ROOT/'outputs/ik_spawn_camera_check/verified_layout'
    output.mkdir(parents=True,exist_ok=True)
    results={}
    for name,seed in [('near_left',744),('near_right',85),('outer_right',183),('far_left',68)]:
        path=output/name;path.mkdir(exist_ok=True)
        with (path/'isaac.log').open('w') as log:
            result=subprocess.run([sys.executable,'-u',__file__,'--headless','--device','cuda:0',
                '--record','--steps','120','--scene-seed',str(seed),'--action-port','15465',
                '--session-port','15466','--video-endpoint','tcp://127.0.0.1:15585',
                '--dataset-root',str(path/'dataset'),'--snapshot-dir',str(path)],
                cwd=ROOT,env=dict(os.environ,PYTHONUNBUFFERED='1'),stdout=log,stderr=subprocess.STDOUT,timeout=90)
        if result.returncode!=0 or not (path/'camera_validation.json').exists():
            raise RuntimeError(f'Camera check failed: {name}; see {path}/isaac.log')
        results[name]=json.loads((path/'camera_validation.json').read_text())
        print(name,results[name],flush=True)
    (output/'validation.json').write_text(json.dumps(results,indent=2)+'\n')
    print('IK SPAWN CAMERA SUITE PASSED',output,flush=True)


def single():
    import cv2
    import numpy as np
    from robot.f14_config import HOME_Q
    from simulation import isaac_teleop_app as app
    original=app.run
    def checked(simulation_app,args,**kwargs):
        states=[]
        def capture(observation,action,sim_time):
            if sim_time>=1.:
                states.append(observation['observation.state'][0].detach().cpu().numpy().copy())
        original(simulation_app,args,observation_callback=capture,**kwargs)
        states=np.asarray(states)
        assert len(states)>10
        error=float(np.max(np.abs(states[:,:14]-HOME_Q)))
        opening=float(states[:,14:].min())
        assert error<.015 and opening>.97,(error,opening)
        image=cv2.imread(str(args.snapshot_dir/'front.png'))
        b,g,r=image[:,:,0].astype(float),image[:,:,1].astype(float),image[:,:,2].astype(float)
        mask=(r>90)&(r>g*1.8)&(r>b*1.8)
        y,x=np.where(mask)
        assert len(x)>25,'Cube is occluded or outside the actual body camera'
        result={'passed':True,'home_max_error_rad':error,'gripper_min_open_ratio':opening,
                'red_cube_pixels':len(x),'red_bbox_xyxy':[int(x.min()),int(y.min()),int(x.max()),int(y.max())]}
        (args.snapshot_dir/'camera_validation.json').write_text(json.dumps(result,indent=2)+'\n')
        print('CAMERA/HOME CHECK PASSED',result,flush=True)
    app.run=checked
    app.main()


if __name__=='__main__':
    if '--suite' in sys.argv:
        suite()
    else:
        single()
