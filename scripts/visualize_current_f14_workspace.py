"""Current free-orientation fingertip workspace using scikit-robot FK.

Run with ~/stkim_ws/.venv/bin/python. Samples are kinematic positions within
URDF joint limits, not collision-free paths or guaranteed live IK convergence.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
from scipy.stats import qmc
from visualize_arm_workspace import load_robot, set_home, box_vertices
from robot.f14_config import ARM_JOINT_NAMES, EE_BODY_NAMES, GRIPPER_TIP_OFFSET, HOME_Q, F14_URDF_PATH
from config import tabletop_config as table
from simulation.tabletop import box_parts, cube_spawn_regions, table_surface_height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=16384, help="Legal FK samples per arm")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/arm_workspace/current_f14")
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    robot = load_robot(output)
    joints = {j.name: j for j in robot.joint_list}
    links = {l.name: l for l in robot.link_list}
    clouds, skeletons, arrays = [], [], {}
    summary = {"library": "scikit-robot", "urdf": str(F14_URDF_PATH),
               "point": "midpoint of gripper ends (TCP)", "tcp_offset_m": GRIPPER_TIP_OFFSET.tolist(),
               "orientation": "free", "collision_checked": False, "path_checked": False,
               "samples_per_arm": args.samples, "seed": args.seed, "home_q_rad": HOME_Q.tolist(),
               "table_center_m": table.TABLE_CENTER, "table_size_m": table.TABLE_SIZE,
               "box_center_xy_m": table.BOX_CENTER_XY, "spawn_regions_m": cube_spawn_regions(), "arms": {}}
    for side, name in enumerate(("left", "right")):
        set_home(joints)
        link = links[EE_BODY_NAMES[side]]
        tip = link.worldpos() + link.worldrot() @ GRIPPER_TIP_OFFSET
        skeletons.append(np.vstack(([links[f"{name}_dof{i}_link"].worldpos().copy() for i in range(1, 8)], tip)))
        arm = [joints[n] for n in ARM_JOINT_NAMES[side * 7:(side + 1) * 7]]
        lower = np.array([j.min_angle for j in arm])
        upper = np.array([j.max_angle for j in arm])
        uniform = qmc.Sobol(7, scramble=True, seed=args.seed + side).random_base2(
            int(np.ceil(np.log2(args.samples))))[:args.samples]
        qs = qmc.scale(uniform, lower, upper)
        points = np.empty((args.samples, 3))
        for i, q in enumerate(qs):
            for joint, angle in zip(arm, q):
                joint.joint_angle(float(angle))
            points[i] = link.worldpos() + link.worldrot() @ GRIPPER_TIP_OFFSET
        assert np.isfinite(points).all() and np.all(qs >= lower) and np.all(qs <= upper)
        clouds.append(points)
        arrays.update({f"{name}_tcp_xyz_m": points, f"{name}_joint_angles_rad": qs})
        summary["arms"][name] = {"home_tcp_m": tip.tolist(),
            "xyz_sample_bounds_m": np.column_stack((points.min(0), points.max(0))).tolist(),
            "joint_limits_rad": np.column_stack((lower, upper)).tolist()}
        print(f"{name}: {len(points)} legal joint samples; HOME TCP={tip.round(6)}", flush=True)
    np.savez_compressed(output / "samples.npz", **arrays)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    render(output, clouds, skeletons, args.samples)
    print(f"Image: {output / 'workspace.png'}", flush=True)


def render(output, clouds, skeletons, count):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    colors = ("#e98616", "#2489d8")
    fig = plt.figure(figsize=(16, 12), layout="constrained", facecolor="#fafbfd")
    fig.suptitle(f"F14 gripper-tip workspace | scikit-robot | {count:,} joint samples per arm\n"
                 "Current URDF + original HOME | Free orientation", fontsize=20, weight="bold")
    ax = fig.add_subplot(2, 2, 1, projection="3d")
    for points, skeleton, color, name in zip(clouds, skeletons, colors, ("Left arm", "Right arm")):
        ax.scatter(*points.T, color=color, s=1, alpha=.11, rasterized=True, label=name)
        ax.plot(*skeleton.T, color=color, linewidth=3)
        ax.scatter(*skeleton[-1], color=color, edgecolor="black", s=130, marker="*", zorder=8)
    ax.plot([0, 0], [0, 0], [0, .85], color="#252a31", linewidth=8)
    faces = ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    solids = [(table.TABLE_CENTER, table.TABLE_SIZE, "#956c43")]
    solids += [(pos, size, "#88bd88") for _, size, pos in box_parts()]
    for center, size, color in solids:
        vertices = box_vertices(center, size)
        ax.add_collection3d(Poly3DCollection([vertices[list(f)] for f in faces],
            facecolor=color, edgecolor=color, alpha=.2))
    ax.set(xlabel="X forward (m)", ylabel="Y left (m)", zlabel="Z up (m)", title="3D reach + HOME arms + table / open box")
    all_points = np.vstack(clouds + skeletons + [np.array([[0, 0, 0], [0, 0, .85]]),
                                                box_vertices(table.TABLE_CENTER, table.TABLE_SIZE)])
    lo, hi = all_points.min(0) - .05, all_points.max(0) + .05
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo); ax.view_init(elev=24, azim=38)
    ax.legend(loc="upper left", markerscale=5)
    cx, cy, cz = table.TABLE_CENTER
    sx, sy, sz = table.TABLE_SIZE
    for subplot, axes, title in ((2, (0, 1), "Top view (XY)"),
                                  (3, (1, 2), "Front view (YZ)"),
                                  (4, (0, 2), "Side view (XZ)")):
        ax = fig.add_subplot(2, 2, subplot)
        a, b = axes
        for points, skeleton, color, name in zip(clouds, skeletons, colors, ("Left arm", "Right arm")):
            ax.scatter(points[:, a], points[:, b], color=color, s=1, alpha=.10, rasterized=True, label=name)
            ax.plot(skeleton[:, a], skeleton[:, b], color=color, linewidth=2.5)
            ax.scatter(skeleton[-1, a], skeleton[-1, b], s=130, marker="*", color=color, edgecolor="black", zorder=8)
        bounds = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2],
                           [cx + sx / 2, cy + sy / 2, cz + sz / 2]])
        ax.add_patch(Rectangle(bounds[0, [a, b]], *(bounds[1] - bounds[0])[[a, b]],
                              facecolor="#956c43", edgecolor="#725031", alpha=.15, label="Table"))
        for _, size, pos in box_parts():
            size, pos = np.array(size), np.array(pos)
            ax.add_patch(Rectangle((pos - size / 2)[[a, b]], *size[[a, b]],
                                  facecolor="#88bd88", edgecolor="#4e874e", alpha=.4))
        if axes == (0, 1):
            for i, (xmin, xmax, ymin, ymax) in enumerate(cube_spawn_regions()):
                ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                    fill=False, edgecolor="#d63535", linestyle="--", linewidth=2,
                    label="Cube placement region (footprint)" if i == 0 else None))
        else:
            ax.axhline(table_surface_height(), color="#725031", linestyle="--", linewidth=1, label="Table top (z=0.30 m)")
        ax.scatter(0, 0, color="#252a31", marker="+", s=80)
        ax.set(xlabel=("X forward (m)", "Y left (m)", "Z up (m)")[a],
               ylabel=("X forward (m)", "Y left (m)", "Z up (m)")[b], title=title,
               xlim=(lo[a], hi[a]), ylim=(lo[b], hi[b]))
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2)
        ax.legend(loc="upper left", fontsize=8, markerscale=4)
    fig.supxlabel("Stars = current HOME fingertips. Full 3D workspace projected in XY / YZ / XZ.\n"
                  "Kinematic samples only: collisions, self-collisions, paths and live controller limits are NOT checked.", fontsize=11)
    fig.savefig(output / "workspace.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
