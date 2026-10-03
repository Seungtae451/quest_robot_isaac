"""Read-only scikit-robot workspace sampling: interactive HTML + overview PNG.

Run with ~/stkim_ws/.venv/bin/python (scikit-robot), not Isaac's interpreter.
Points represent left/right_dof7_link origins, NOT the fingertip positions.
This is sampled kinematic reachability; collisions and paths are not checked.
"""
import argparse
import io
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

os.environ.setdefault("MPLCONFIGDIR", "/tmp/f14-workspace-matplotlib")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from scipy.stats import qmc
from skrobot.coordinates import Coordinates
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode

from robot.f14_config import (ARM_JOINT_NAMES, EE_BODY_NAMES, F14_URDF_PATH,
    HOME_Q, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT)
from config import tabletop_config as table


def load_robot(output, meshes=False):
    # Resolve ROS package:// URLs in a COPY; never alter the user's URDF/assets.
    tree = ET.parse(F14_URDF_PATH)
    for mesh in tree.findall(".//mesh"):
        path = F14_URDF_PATH.parent.parent / "meshes" / mesh.get("filename").split("/")[-1]
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set("filename", str(path))
    resolved = output / "resolved_robot.urdf"
    tree.write(resolved, encoding="utf-8", xml_declaration=True)
    robot = RobotModel()
    if meshes:
        robot.load_urdf_file(str(resolved))
    else:
        with no_mesh_load_mode():
            robot.load_urdf_file(io.BytesIO(ET.tostring(tree.getroot())))
    return robot


def set_home(joints):
    for name, angle in zip(ARM_JOINT_NAMES, HOME_Q):
        joints[name].joint_angle(angle)


def grid(step, height=None):
    axes = [np.arange(.10, .701, step), np.arange(-.10, .551, step),
            np.arange(.30, .851, step) if height is None else [height]]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def downward_ik(robot, joints, links, side, points, args):
    set_home(joints)
    names = ARM_JOINT_NAMES[:7] if side == 0 else ARM_JOINT_NAMES[7:]
    rotation = (LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT)[side]
    targets = [Coordinates(pos=p, rot=rotation) for p in points]
    result = robot.batch_inverse_kinematics(targets,
        move_target=links[EE_BODY_NAMES[side]], joint_list=[joints[n] for n in names],
        position_mask=True, rotation_mask=True, stop=args.iterations,
        thre=.001, rthre=.002, initial_angles="current", backend="numpy",
        attempts_per_pose=args.attempts, random_initial_range=.6, retry_seed="random")
    flags = np.asarray(result.success_flags, dtype=bool)
    # Independently check returned solutions with the actual loaded robot FK.
    # Reject any solver flag whose pose or joint limits fail the stated checks.
    order = [j.name for j in robot.joint_list]
    lower = np.array([j.min_angle for j in robot.joint_list])
    upper = np.array([j.max_angle for j in robot.joint_list])
    solutions = np.full((len(points), 7), np.nan)
    errors = []
    for i in np.flatnonzero(flags):
        # The batch backend returns float32; feeding it to incremental joint
        # rotations accumulates float32 trig roundoff across thousands of poses.
        # Use float64 for the independent model/FK verification.
        q = np.asarray(result.solutions[i], dtype=np.float64)
        if not (np.all(q >= lower - 1e-6) and np.all(q <= upper + 1e-6)):
            flags[i] = False
            continue
        # Correct only float32 rounding at a limit (<1 microradian), then
        # validate FK again at the strictly legal angles actually stored.
        q = np.clip(q, lower, upper)
        robot.angle_vector(q)
        ee = links[EE_BODY_NAMES[side]]
        position_error = np.linalg.norm(ee.worldpos() - points[i])
        relative = rotation.T @ ee.worldrot()
        rotation_error = np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1))
        flags[i] = (position_error <= .0011 and rotation_error <= .0021
                    and np.all(q >= lower - 1e-6) and np.all(q <= upper + 1e-6))
        if flags[i]:
            solutions[i] = q[[order.index(n) for n in names]]
            errors.append([position_error, rotation_error])
    print(f"{'Left' if side == 0 else 'Right'} downward: {flags.sum()}/{len(points)} IK verified", flush=True)
    return flags, solutions, np.asarray(errors)


def box_vertices(center, size):
    signs = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                      [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
    return np.asarray(center) + signs * np.asarray(size) / 2


def figures(output, robot, joints, links, free, downward, plane, flags, args):
    import plotly.graph_objects as go
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    set_home(joints)
    colors = ["#e69f00", "#0072b2"]
    fig = go.Figure()
    fixed_indices, cloud_indices, down_indices, spawn_indices = [], [], [], []
    # A lightweight HOME preview from link positions avoids huge STL payloads.
    for side in range(2):
        prefix = ("left", "right")[side]
        p = np.array([links[f"{prefix}_dof{i}_link"].worldpos() for i in range(1, 8)])
        tip = p[-1] + links[EE_BODY_NAMES[side]].worldrot() @ [0, -.235, 0]
        p = np.vstack((p, tip))
        fixed_indices.append(len(fig.data))
        fig.add_trace(go.Scatter3d(x=p[:,0], y=p[:,1], z=p[:,2], mode="lines+markers",
            line=dict(color=colors[side], width=9), marker=dict(size=4), name=f"{prefix} HOME arm / fingertip"))
    fixed_indices.append(len(fig.data))
    fig.add_trace(go.Scatter3d(x=[0,0], y=[0,0], z=[0,.85], mode="lines",
                              line=dict(width=16,color="#6e7480"), name="body reference"))
    # Include an actual URDF silhouette in HTML without millions of triangles:
    # each visual link is represented by its convex hull at HOME. This is a
    # display approximation only, never a collision/reachability calculation.
    mesh_robot = load_robot(output, meshes=True)
    set_home({j.name:j for j in mesh_robot.joint_list})
    for link in mesh_robot.link_list:
        for mesh in link.visual_mesh or []:
            hull = mesh.convex_hull
            vertices = hull.vertices @ link.worldrot().T + link.worldpos()
            faces = hull.faces
            from robot.appearance import link_color
            rgb = link_color(link.name)
            color = "#" + "".join(f"{round(c*255):02x}" for c in rgb)
            fixed_indices.append(len(fig.data))
            fig.add_trace(go.Mesh3d(x=vertices[:,0],y=vertices[:,1],z=vertices[:,2],
                i=faces[:,0],j=faces[:,1],k=faces[:,2],color=color,opacity=1.0,
                name=f"HOME {link.name} (visual hull)",showlegend=False))
    faces = np.array([[0,1,2],[0,2,3],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                      [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
    z = table.TABLE_CENTER[2] + table.TABLE_SIZE[2]/2
    for name, center, size, color in (
        ("table",table.TABLE_CENTER,table.TABLE_SIZE,"#966d43"),
        ("open box footprint",(*table.BOX_CENTER_XY,z+table.BOX_SIZE[2]/2),table.BOX_SIZE,"#8ccc8c")):
        v = box_vertices(center,size)
        fixed_indices.append(len(fig.data))
        fig.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=faces[:,0],j=faces[:,1],k=faces[:,2],
            opacity=.25,color=color,name=name,showlegend=True))
    for side, name in enumerate(("Left", "Right")):
        for p, indices, title, opacity, size in ((free[side],cloud_indices,"free orientation",.13,2),
                (downward[side],down_indices,"fixed downward orientation",.85,4)):
            indices.append(len(fig.data))
            fig.add_trace(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode="markers",
                marker=dict(size=size,color=colors[side],opacity=opacity),name=f"{name}: {title}",
                hovertemplate=f"{name} wrist<br>X %{{x:.3f}} m<br>Y %{{y:.3f}} m<br>Z %{{z:.3f}} m<extra>{title}</extra>"))
    # The exact scene spawn pool uses the REAL teleop solver and route checks,
    # independently of this scikit-robot reachability exploration.
    from robot.spawn_workspace import CACHE_PATH, workspace_signature
    spawn_points=np.empty((0,3))
    if CACHE_PATH.is_file():
        pool=json.loads(CACHE_PATH.read_text())
        if pool.get('signature')==workspace_signature():
            spawn_points=np.array([[p['x'],p['y'],z+table.CUBE_SIZE/2] for p in pool['candidates']])
            spawn_indices.append(len(fig.data))
            fig.add_trace(go.Scatter3d(x=spawn_points[:,0],y=spawn_points[:,1],z=spawn_points[:,2],
                mode='markers',marker=dict(size=4,color='#2ca02c'),name='Cube spawn centers: teleop IK route verified',
                hovertemplate='Cube center<br>X %{x:.3f} m<br>Y %{y:.3f} m<br>Z %{z:.3f} m<extra>Verified spawn location</extra>'))
    def visibility(selected):
        return [i in fixed_indices or i in spawn_indices or i in selected for i in range(len(fig.data))]
    fig.update_layout(title="F14 wrist workspace — scikit-robot (kinematic samples, no collision/path check)",
        scene=dict(xaxis_title="X forward (m)",yaxis_title="Y left (m)",zaxis_title="Z up (m)",
            aspectmode="data",camera=dict(eye=dict(x=1.6,y=1.6,z=1.1))),height=800,
        margin=dict(l=0,r=0,b=0,t=100),legend=dict(x=0,y=1),
        updatemenus=[dict(x=.45,y=1.12,buttons=[
            dict(label="Both modes",method="update",args=[{"visible":visibility(cloud_indices+down_indices)}]),
            dict(label="Downward only",method="update",args=[{"visible":visibility(down_indices)}]),
            dict(label="Verified spawn locations",method="update",args=[{"visible":visibility([])}]),
            dict(label="Free orientation only",method="update",args=[{"visible":visibility(cloud_indices)}])])])
    html = fig.to_html(full_html=True,include_plotlyjs=True)
    note = '<p style="font:16px sans-serif;margin:20px">마우스 드래그: 회전 · 휠: 확대 · 메뉴: 자유 방향/수직 고정 비교 · 범례 클릭: 팔 표시/숨김.<br>점은 손목 링크의 위치입니다. 손가락 끝은 수직일 때 약 23.5cm 아래입니다. 충돌·이동 경로·현재 teleop solver 성공 여부는 이 그림으로 보장하지 않습니다.</p>'
    (output / "workspace.html").write_text(html.replace("<body>","<body>"+note),encoding="utf-8")

    preview = plt.figure(figsize=(17,6),constrained_layout=True)
    for column, (clouds,title) in enumerate(((free,"Free orientation: joint-limit FK samples"),
                                           (downward,"Downward fixed yaw: IK-verified grid")),1):
        ax=preview.add_subplot(1,3,column,projection="3d")
        for side,name in enumerate(("Left","Right")):
            p=clouds[side]
            ax.scatter(p[:,0],p[:,1],p[:,2],s=2 if column==1 else 8,alpha=.18 if column==1 else .8,
                       color=colors[side],label=name,rasterized=True)
            prefix=name.lower()
            home=np.array([links[f"{prefix}_dof{i}_link"].worldpos() for i in range(1,8)])
            ax.plot(*home.T,color=colors[side],linewidth=3)
            wrist=home[-1]
            ax.scatter(*wrist,color=colors[side],s=55,marker="*")
            ax.plot([wrist[0]]*2,[wrist[1]]*2,[wrist[2],wrist[2]-.235],color=colors[side],linewidth=2)
        cx,cy,_=table.TABLE_CENTER;sx,sy,_=table.TABLE_SIZE
        tx=cx+np.array([-1,1,1,-1,-1])*sx/2
        ty=cy+np.array([-1,-1,1,1,-1])*sy/2
        ax.plot(tx,ty,np.full(5,z),color="#966d43")
        ax.set(xlabel="X forward (m)",ylabel="Y left (m)",zlabel="Z up (m)",title=title)
        ax.set_xlim(-.6,.9);ax.set_ylim(-.9,.9);ax.set_zlim(-.2,1.4)
        ax.set_box_aspect((1.5,1.8,1.6));ax.view_init(elev=22,azim=35);ax.legend(loc="upper left")
    ax=preview.add_subplot(1,3,3)
    for side,name in enumerate(("Left","Right")):
        p=plane[side];found=flags[side]
        ax.scatter(p[~found,0],p[~found,1],s=10,c="#cccccc",marker="x",label="IK not found" if side==0 else None)
        ax.scatter(p[found,0],p[found,1],s=22,c=colors[side],label=name)
    cx,cy,_=table.TABLE_CENTER;sx,sy,_=table.TABLE_SIZE
    ax.add_patch(Rectangle((cx-sx/2,cy-sy/2),sx,sy,fill=False,color="#966d43",linewidth=2,label="Table"))
    bx,by=table.BOX_CENTER_XY;bsx,bsy,_=table.BOX_SIZE
    ax.add_patch(Rectangle((bx-bsx/2,by-bsy/2),bsx,bsy,facecolor="#8ccc8c",alpha=.3,label="Box"))
    for sign in [-1,1]:
        x0=cx-sx/2+sx*table.CUBE_SPAWN_START_FRACTION;x1=cx-sx/2+sx*table.CUBE_SPAWN_LENGTH_FRACTION
        inner=bsy/2+table.BOX_SPAWN_CLEARANCE;outer=sy/2-table.CUBE_TABLE_EDGE_MARGIN
        ax.add_patch(Rectangle((x0,inner if sign==1 else -outer),x1-x0,outer-inner,
            fill=False,linestyle="--",edgecolor="#bc3b3b",label="Cube spawn strip" if sign==1 else None))
    if len(spawn_points):
        ax.scatter(spawn_points[:,0],spawn_points[:,1],s=24,facecolors='none',edgecolors='#2ca02c',
                   marker='s',linewidths=.8,label='Verified cube centers')
    ax.set(xlabel="X forward (m)",ylabel="Y left (m)",title=f"Downward wrist height Z={args.slice_height:.2f} m\nGray = no solution found (not proof of impossibility)")
    ax.set_aspect("equal")
    ax.legend(fontsize=8,loc="upper center",bbox_to_anchor=(.5,-.16),ncol=2)
    ax.grid(alpha=.2)
    preview.suptitle("F14 reach samples • wrist positions • joint limits only • no collision/path checks",fontsize=15)
    preview.savefig(output / "workspace.png",dpi=170)
    plt.close(preview)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples",type=int,default=4096,help="FK samples per arm")
    parser.add_argument("--grid-step",type=float,default=.075,help="Downward 3D grid spacing (m)")
    parser.add_argument("--slice-height",type=float,default=.54,help="Wrist height for table XY plot (m)")
    parser.add_argument("--slice-step",type=float,default=.025)
    parser.add_argument("--attempts",type=int,default=4,help="HOME seed plus randomized IK retries")
    parser.add_argument("--iterations",type=int,default=200)
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/arm_workspace")
    parser.add_argument("--viewer",action="store_true",help="Also open native scikit-robot URDF HOME viewer")
    args=parser.parse_args()
    if args.samples<1 or args.grid_step<=0 or args.slice_step<=0 or args.attempts<1 or args.iterations<1:
        parser.error("Sample count, spacing, attempts and iterations must be positive")
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    np.random.seed(args.seed)
    robot=load_robot(output)
    joints={j.name:j for j in robot.joint_list};links={l.name:l for l in robot.link_list}
    set_home(joints)
    for side,expected in enumerate(([.4,.18,.57],[.4,-.18,.57])):
        np.testing.assert_allclose(links[EE_BODY_NAMES[side]].worldpos(),expected,atol=1e-8)
    free=[];down=[];planes=[];plane_flags=[];arrays={};summary={"urdf":str(F14_URDF_PATH),
        "point":"left/right_dof7_link wrist origin", "collision_checked":False,
        "path_checked":False,"ik_position_tolerance_m":.001,"ik_rotation_tolerance_rad":.002,
        "settings":{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},"arms":{}}
    for side,name in enumerate(("left","right")):
        set_home(joints)
        names=ARM_JOINT_NAMES[side*7:(side+1)*7];arm=[joints[n] for n in names]
        lower=np.array([j.min_angle for j in arm]);upper=np.array([j.max_angle for j in arm])
        samples=qmc.Sobol(7,scramble=True,seed=args.seed+side).random_base2(int(np.ceil(np.log2(args.samples))))[:args.samples]
        qs=qmc.scale(samples,lower,upper);points=[]
        for q in qs:
            for j,a in zip(arm,q):j.joint_angle(a)
            points.append(links[EE_BODY_NAMES[side]].worldpos().copy())
        points=np.asarray(points);free.append(points)
        targets=grid(args.grid_step);plane=grid(args.slice_step,args.slice_height)
        if side==1:
            targets[:,1]*=-1;plane[:,1]*=-1
        all_points=np.vstack((targets,plane))
        flags,solutions,errors=downward_ik(robot,joints,links,side,all_points,args)
        down.append(targets[flags[:len(targets)]]);planes.append(plane);plane_flags.append(flags[len(targets):])
        arrays.update({f"{name}_free_xyz":points,f"{name}_free_q":qs,
            f"{name}_down_grid":targets,f"{name}_down_found":flags[:len(targets)],
            f"{name}_slice_grid":plane,f"{name}_slice_found":flags[len(targets):],
            f"{name}_down_q":solutions[:len(targets)],f"{name}_slice_q":solutions[len(targets):]})
        summary["arms"][name]={"joint_limits_deg":np.rad2deg(np.column_stack((lower,upper))).tolist(),
            "free_sample_xyz_bounds_m":np.column_stack((points.min(0),points.max(0))).tolist(),
            "downward_found":len(down[-1]),"downward_tested":len(targets),
            "slice_found":int(plane_flags[-1].sum()),"slice_tested":len(plane),
            "verified_max_error_m_rad":errors.max(0).tolist() if len(errors) else None}
    np.savez_compressed(output/"samples.npz",**arrays)
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    figures(output,robot,joints,links,free,down,planes,plane_flags,args)
    print(f"Workspace generated: {output/'workspace.html'}\nOverview: {output/'workspace.png'}",flush=True)
    if args.viewer:
        import trimesh
        from skrobot.viewers import TrimeshSceneViewer
        robot=load_robot(output,meshes=True)
        set_home({j.name:j for j in robot.joint_list})
        viewer=TrimeshSceneViewer(resolution=(1200,800));viewer.add(robot)
        for side,color in enumerate(([230,159,0,255],[0,114,178,255])):
            viewer.scene.add_geometry(trimesh.points.PointCloud(down[side],colors=color))
        transform=np.eye(4);transform[:3,3]=table.TABLE_CENTER
        table_mesh=trimesh.creation.box(extents=table.TABLE_SIZE,transform=transform)
        table_mesh.visual.face_colors=[150,109,67,90]
        viewer.scene.add_geometry(table_mesh)
        viewer.show()
        while viewer.is_active:
            time.sleep(.05)


if __name__=="__main__":
    main()
